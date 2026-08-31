#!/usr/bin/env python3
"""Visualizes ESP32 room scan data from serial or a demo stream."""

from __future__ import annotations

import argparse
import atexit
import csv
import datetime as dt
import json
import logging
import os
import queue
import threading
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime with a clear message.
    serial = None

import matplotlib
from matplotlib.animation import FuncAnimation

LOGGER = logging.getLogger("room_scanner")
plt = None

DEFAULT_PORTS = [
    "COM10", #change to fit your port
    "COM3",
    "/dev/ttyUSB0",
    "/dev/tty.usbserial",
    "/dev/ttyACM0",
]


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Monitor ESP32 room scan data and plot a live radar and map."
    )
    parser.add_argument("--port", default=os.getenv("ESP32_PORT"), help="Serial port to read from.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--max-dist", type=float, default=1200.0, help="Radar/map limit in millimeters.")
    parser.add_argument("--max-points", type=int, default=5000, help="Maximum points in the room map.")
    parser.add_argument("--update-rate", type=int, default=50, help="Animation refresh interval in milliseconds.")
    parser.add_argument("--demo", action="store_true", help="Generate synthetic data instead of reading serial input.")
    parser.add_argument("--headless", action="store_true", help="Use a non-interactive matplotlib backend.")
    parser.add_argument("--frame-count", type=int, default=0, help="Process a fixed number of frames and exit. Useful for tests and headless runs.")
    parser.add_argument("--no-save", action="store_true", help="Do not save scan data to CSV on exit.")
    return parser


def is_valid_sample(data: Dict[str, Any]) -> bool:
    if not isinstance(data, dict):
        return False
    angle = data.get("angle")
    return angle is not None


def normalize_sample(data: Dict[str, Any]) -> Dict[str, float]:
    sample = {
        "angle": float(data.get("angle", 0.0)),
        "dist": float(data.get("dist", 0.0)),
        "lat": float(data.get("lat", 0.0)),
        "lon": float(data.get("lon", 0.0)),
        "temp": float(data.get("temp", 0.0)),
        "pitch": float(data.get("pitch", 0.0)),
        "roll": float(data.get("roll", 0.0)),
        "yaw": float(data.get("yaw", 0.0)),
    }
    sample["angle"] = int(sample["angle"])
    if sample["dist"] < 0:
        sample["dist"] = 0.0
    return sample


class RoomScannerApp:
    def __init__(self, port: Optional[str], baud: int, max_dist: float, max_points: int, update_rate: int, no_save: bool):
        self.port = port
        self.baud = baud
        self.max_dist = max_dist
        self.max_points = max_points
        self.update_rate = update_rate
        self.no_save = no_save
        self.data_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.radar_dist = [0.0] * 181
        self.map_points: Deque[Tuple[float, float, float]] = deque(maxlen=max_points)
        self.latest = {
            "lat": 0.0,
            "lon": 0.0,
            "temp": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
        }
        self.csv_rows: List[Dict[str, Any]] = []
        self._serial_thread: Optional[threading.Thread] = None
        self._demo_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._build_plot()
        atexit.register(self.save_csv)

    def _build_plot(self) -> None:
        self.fig = plt.figure(figsize=(14, 7))
        self.fig.patch.set_facecolor("#0d1117")

        self.ax_radar = self.fig.add_subplot(121, projection="polar")
        self.ax_radar.set_facecolor("#161b22")
        self.ax_radar.set_theta_zero_location("N")
        self.ax_radar.set_theta_direction(-1)
        self.ax_radar.set_rlim(0, self.max_dist)
        self.ax_radar.grid(color="#30363d", linestyle="-", linewidth=0.5)

        angle_ticks = np.radians([0, 45, 90, 135, 180])
        self.ax_radar.set_xticks(angle_ticks)
        self.ax_radar.set_xticklabels(["0°", "45°", "90°", "135°", "180°"], color="#c9d1d9")
        self.ax_radar.set_yticks(np.linspace(0, self.max_dist, 5))
        self.ax_radar.set_yticklabels([f"{int(v)}mm" for v in np.linspace(0, self.max_dist, 5)], color="#c9d1d9")
        self.ax_radar.set_title("Live Radar", color="#c9d1d9", fontsize=14, pad=20)

        self.angles_rad = np.radians(np.arange(0, 181))
        self.radar_line, = self.ax_radar.plot([], [], "o-", color="#58a6ff", markersize=4, linewidth=2)
        self.radar_fill = self.ax_radar.fill_between(self.angles_rad, 0, 0, color="#58a6ff", alpha=0.25)

        self.ax_map = self.fig.add_subplot(122)
        self.ax_map.set_facecolor("#161b22")
        self.ax_map.set_xlim(-self.max_dist, self.max_dist)
        self.ax_map.set_ylim(-self.max_dist, self.max_dist)
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(color="#30363d", linestyle="-", linewidth=0.5)
        self.ax_map.set_xlabel("X (mm)", color="#c9d1d9")
        self.ax_map.set_ylabel("Y (mm)", color="#c9d1d9")
        self.ax_map.set_title("Room Map", color="#c9d1d9", fontsize=14, pad=20)
        self.map_scatter = self.ax_map.scatter([], [], c=[], cmap="viridis", s=8, alpha=0.7, vmin=0, vmax=self.max_dist)

        self.info_text = self.ax_map.text(
            0.05,
            0.95,
            "",
            transform=self.ax_map.transAxes,
            color="#c9d1d9",
            fontsize=10,
            verticalalignment="top",
            bbox={"facecolor": "#0d1117", "edgecolor": "#30363d", "alpha": 0.8},
        )

    def start_serial_reader(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install it with: pip install pyserial")

        def worker() -> None:
            try:
                serial_port = self.port or auto_detect_port()
                if serial_port is None:
                    LOGGER.warning("No serial port detected. Use --port or --demo mode.")
                    return

                with serial.Serial(serial_port, self.baud, timeout=1) as ser:
                    LOGGER.info("Connected to %s", serial_port)
                    while not self._stop_event.is_set():
                        line = ser.readline()
                        if not line:
                            continue
                        try:
                            payload = json.loads(line.decode("utf-8", errors="replace").strip())
                        except json.JSONDecodeError:
                            continue
                        if is_valid_sample(payload):
                            self.data_queue.put(normalize_sample(payload))
            except serial.SerialException as exc:
                LOGGER.error("Serial error on %s: %s", self.port or "auto-detect", exc)
                LOGGER.error("Check that the port is correct and no other app is using it.")
            except Exception as exc:  # pragma: no cover - defensive guard.
                LOGGER.exception("Unexpected serial-reader failure: %s", exc)

        self._serial_thread = threading.Thread(target=worker, name="serial-reader", daemon=True)
        self._serial_thread.start()

    def start_demo_stream(self) -> None:
        def worker() -> None:
            angle = 0
            while not self._stop_event.is_set():
                sample = {
                    "angle": angle,
                    "dist": max(60, int(500 + 400 * np.sin(np.deg2rad(angle)) + (20 * np.random.rand()))),
                    "lat": 43.3209 + 0.0001 * np.sin(np.deg2rad(angle * 4)),
                    "lon": 21.8958 + 0.0001 * np.cos(np.deg2rad(angle * 3)),
                    "temp": 22.0 + 4.0 * np.sin(np.deg2rad(angle)),
                    "pitch": np.sin(np.deg2rad(angle)) * 20,
                    "roll": np.cos(np.deg2rad(angle * 1.5)) * 15,
                    "yaw": angle,
                }
                self.data_queue.put(normalize_sample(sample))
                angle = (angle + 2) % 181
                self._stop_event.wait(0.05)

        self._demo_thread = threading.Thread(target=worker, name="demo-stream", daemon=True)
        self._demo_thread.start()

    def consume_queue(self) -> None:
        while not self.data_queue.empty():
            payload = self.data_queue.get()
            if not is_valid_sample(payload):
                continue

            sample = normalize_sample(payload)
            angle = sample["angle"]
            dist = sample["dist"]

            if 0 <= angle <= 180:
                self.radar_dist[angle] = dist

            self.latest.update({
                "lat": sample["lat"],
                "lon": sample["lon"],
                "temp": sample["temp"],
                "pitch": sample["pitch"],
                "roll": sample["roll"],
                "yaw": sample["yaw"],
            })

            angle_rad = np.radians(angle)
            x = dist * np.cos(angle_rad)
            y = dist * np.sin(angle_rad)
            self.map_points.append((x, y, dist))
            self.csv_rows.append({
                "angle": angle,
                "dist": dist,
                "pitch": sample["pitch"],
                "roll": sample["roll"],
                "yaw": sample["yaw"],
                "temp": sample["temp"],
                "lat": sample["lat"],
                "lon": sample["lon"],
            })

    def update_frame(self, _frame: int) -> Iterable[object]:
        self.consume_queue()

        dists_closed = np.array(self.radar_dist[:181] + [self.radar_dist[0]], dtype=float)
        angles_closed = np.append(self.angles_rad, self.angles_rad[0])

        self.radar_line.set_data(angles_closed, dists_closed)
        self.radar_fill.remove()
        self.radar_fill = self.ax_radar.fill_between(angles_closed, 0, dists_closed, color="#58a6ff", alpha=0.25)

        if self.map_points:
            xs, ys, cols = zip(*self.map_points)
            self.map_scatter.set_offsets(np.c_[xs, ys])
            self.map_scatter.set_array(np.array(cols))
        else:
            self.map_scatter.set_offsets(np.empty((0, 2)))
            self.map_scatter.set_array(np.array([]))

        self.info_text.set_text(
            f"GPS: {self.latest['lat']:.6f}, {self.latest['lon']:.6f}\n"
            f"Temp: {self.latest['temp']:.1f}°C\n"
            f"Pitch: {self.latest['pitch']:.1f}°  Roll: {self.latest['roll']:.1f}°  Yaw: {self.latest['yaw']:.1f}°\n"
            f"Points: {len(self.map_points)}"
        )

        return self.radar_line, self.radar_fill, self.map_scatter, self.info_text

    def save_csv(self) -> None:
        if self.no_save or not self.csv_rows:
            return

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = Path(f"scan_{timestamp}.csv")
        with filename.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["angle", "dist", "pitch", "roll", "yaw", "temp", "lat", "lon"])
            writer.writeheader()
            writer.writerows(self.csv_rows)
        LOGGER.info("Saved %d scan samples to %s", len(self.csv_rows), filename)

    def run(self, demo: bool = False, frame_count: int = 0) -> None:
        if demo:
            LOGGER.info("Demo mode enabled: synthetic scan data is being used for testing only.")
            self.start_demo_stream()
        else:
            serial_port = self.port or auto_detect_port()
            if serial_port:
                self.port = serial_port
                self.start_serial_reader()
            else:
                raise RuntimeError(
                    "No serial port found. Connect the ESP32 or run with --demo to generate synthetic data."
                )

        if frame_count > 0:
            for _ in range(frame_count):
                self.update_frame(_)
            plt.close(self.fig)
            self._stop_event.set()
            return

        self.fig.tight_layout()
        animation = FuncAnimation(self.fig, self.update_frame, interval=self.update_rate, blit=True, cache_frame_data=False)
        plt.show()
        self._stop_event.set()


def auto_detect_port() -> Optional[str]:
    if serial is None:
        return None

    try:
        from serial.tools import list_ports

        ports = list_ports.comports()
        if not ports:
            return None

        candidates = []
        for port in ports:
            name = port.device.lower()
            description = (port.description or "").lower()
            if any(marker in description for marker in ("arduino", "usb serial", "cp210", "ch340", "esp32")) or "com" in name:
                candidates.append(port.device)

        return candidates[0] if candidates else ports[0].device
    except Exception:  # pragma: no cover - best effort only.
        return None


def main() -> None:
    parser = build_argument_parser()
    args = parser.parse_args()

    if args.headless:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    global plt
    plt = matplotlib.pyplot

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    app = RoomScannerApp(
        port=args.port,
        baud=args.baud,
        max_dist=args.max_dist,
        max_points=args.max_points,
        update_rate=args.update_rate,
        no_save=args.no_save,
    )

    try:
        app.run(demo=args.demo, frame_count=args.frame_count)
    except RuntimeError as exc:
        LOGGER.error(str(exc))
        raise SystemExit(1)
    finally:
        app.save_csv()
        app._stop_event.set()


if __name__ == "__main__":
    main()
