
#!/usr/bin/env python3
# Live spectrum viewer for RP2040_receiver
# Protocol:
#   #BEGIN,15625.00,61.0352
#   256 lines: frequency_Hz,level_dB
#   #END

import argparse
import time
import threading
import queue

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


FFT_N = 1024


def parse_args():
    p = argparse.ArgumentParser(
        description="RP2040_receiver live spectrum"
    )
    p.add_argument("--port", default="COM22",
                   help="Serial port, e.g. COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--center", type=float, default=3600000.0,
                   help="RF frequency in Hz")
    p.add_argument("--relative", action="store_true",
                   help="Show baseband frequency offset")
    p.add_argument("--ymin", type=float, default=-130.0)
    p.add_argument("--ymax", type=float, default=-20.0)
    p.add_argument("--smooth", type=float, default=0.20,
                   help="Smoothing 0..0.99")
    return p.parse_args()


class SpectrumReader:
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.2
        )
        self.frames = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.error = None

        # Firmware may reset the board when the serial port opens.
        time.sleep(1.5)
        self.ser.reset_input_buffer()

        self.thread = threading.Thread(
            target=self._read_loop,
            daemon=True
        )
        self.thread.start()

    def _publish(self, fs, bin_hz, freq, db):
        if not freq:
            return

        frame = (fs, bin_hz, freq[:], db[:], time.monotonic())

        # Keep the newest complete frame, not an old backlog.
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frames.put_nowait(frame)
            except queue.Full:
                pass

    def _read_loop(self):
        in_frame = False
        fs = 1953.125
        bin_hz = fs / FFT_N
        freq = []
        db = []

        try:
            while not self.stop_event.is_set():
                raw = self.ser.readline()
                if not raw:
                    continue

                s = raw.decode("ascii", errors="ignore").strip()
                if not s:
                    continue

                # Start of a spectrum frame.
                if s.startswith("#BEGIN,"):
                    # Recover a complete 256-bin frame if #END was lost.
                    if in_frame and len(freq) == FFT_N and len(db) == FFT_N:
                        self._publish(fs, bin_hz, freq, db)

                    parts = s.split(",")
                    try:
                        fs = float(parts[1])
                        bin_hz = float(parts[2])
                    except (ValueError, IndexError):
                        in_frame = False
                        freq, db = [], []
                        continue

                    in_frame = True
                    freq, db = [], []
                    continue

                if s == "#END":
                    if in_frame and len(freq) == FFT_N and len(db) == FFT_N:
                        self._publish(fs, bin_hz, freq, db)

                    in_frame = False
                    freq, db = [], []
                    continue

                # Ignore startup and status messages.
                if s.startswith("#") or not in_frame:
                    continue

                parts = s.split(",")
                if len(parts) < 2:
                    continue

                try:
                    f = float(parts[0])
                    level = float(parts[1])
                except ValueError:
                    continue

                freq.append(f)
                db.append(level)

                # The firmware sends exactly FFT_N bins. This also allows
                # recovery if an #END marker is missing.
                if len(freq) == FFT_N:
                    self._publish(fs, bin_hz, freq, db)
                    in_frame = False
                    freq, db = [], []

        except (serial.SerialException, OSError) as exc:
            self.error = str(exc)

    def close(self):
        self.stop_event.set()
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    args = parse_args()

    if not 0.0 <= args.smooth < 1.0:
        raise SystemExit("--smooth must be between 0 and 1")

    try:
        reader = SpectrumReader(args.port, args.baud)
    except serial.SerialException as exc:
        raise SystemExit(
            f"Cannot open {args.port}: {exc}\n"
            "Close Arduino Serial Monitor and check the COM port."
        )

    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title(
        "RP2040_receiver — live spectrum"
    )

    line, = ax.plot([], [], linewidth=1.1)
    peak_marker, = ax.plot([], [], "o", markersize=5)
    peak_text = ax.text(
        0.02, 0.96, "", transform=ax.transAxes, va="top"
    )
    status_text = ax.text(
        0.02, 0.02, "Waiting for spectrum data...",
        transform=ax.transAxes, va="bottom"
    )

    ax.set_title("RP2040_receiver — real-time spectrum")
    ax.set_xlabel(
        "Frequency offset (kHz)"
        if args.relative else "RF frequency (MHz)"
    )
    ax.set_ylabel("Level (dB, firmware scale)")
    ax.set_ylim(args.ymin, args.ymax)
    from matplotlib.ticker import MultipleLocator
    ax.yaxis.set_major_locator(MultipleLocator(10))
    ax.grid(True, alpha=0.3)

    current = {
        "y": None,
        "frames": 0,
        "last_time": None,
        "fs": 0.0,
        "bin_hz": 0.0,
        "freq": [],
        "db": [],
    }

    def update(_):
        newest = None

        # Drain queue and use only the newest frame.
        while True:
            try:
                newest = reader.frames.get_nowait()
            except queue.Empty:
                break

        if newest is not None:
            fs, bin_hz, freq, db, stamp = newest

            if args.smooth > 0 and current["y"] is not None:
                prev = current["y"]
                if len(prev) == len(db):
                    y = [
                        args.smooth * old + (1.0 - args.smooth) * new
                        for old, new in zip(prev, db)
                    ]
                else:
                    y = db[:]
            else:
                y = db[:]

            current.update({
                "y": y,
                "frames": current["frames"] + 1,
                "last_time": stamp,
                "fs": fs,
                "bin_hz": bin_hz,
                "freq": freq,
                "db": db,
            })

            if args.relative:
                x = [f / 1000.0 for f in freq]
            else:
                x = [(args.center + f) / 1e6 for f in freq]

            line.set_data(x, y)
            ax.set_xlim(min(x), max(x))

            idx = max(range(len(y)), key=lambda i: y[i])
            peak_marker.set_data([x[idx]], [y[idx]])

            if args.relative:
                peak = f"Peak: {freq[idx] / 1000.0:+.3f} kHz, {y[idx]:.1f} dB"
            else:
                peak = (
                    f"Peak: {(args.center + freq[idx]) / 1e6:.6f} MHz, "
                    f"{y[idx]:.1f} dB"
                )
            peak_text.set_text(peak)

        if current["last_time"] is None:
            status = "Waiting for a complete 256-bin frame..."
        else:
            age = time.monotonic() - current["last_time"]
            status = (
                f"Fs={current['fs']:.2f} Hz | "
                f"bin={current['bin_hz']:.4f} Hz | "
                f"bins={len(current['freq'])} | "
                f"frames={current['frames']} | "
                f"age={age:.1f}s"
            )

        if reader.error:
            status += " | SERIAL ERROR: " + reader.error

        status_text.set_text(status)

        return line, peak_marker, peak_text, status_text

    ani = FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        reader.close()


if __name__ == "__main__":
    main()
