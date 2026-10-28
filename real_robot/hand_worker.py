"""Dedicated can1 worker; no CAN socket is opened for mock mode."""
import importlib
import sys
from pathlib import Path
import threading
import time


class HandWorker:
    def __init__(self, hardware, real=False):
        self.hardware, self.real = hardware, real
        self.ready, self.healthy, self.error = False, True, ""
        self.updated = time.monotonic()
        self.target, self.current = None, None
        self.event = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def run(self):
        if not self.real:
            self.ready = True
            self.updated = time.monotonic()
            return
        transport = None
        try:
            directory = Path(__file__).resolve().parent.parent / "tools/omnihand_can_test"
            sys.path.insert(0, str(directory))
            motion = importlib.import_module("fist_test")
            ro = motion.ro
            import json
            target = json.loads(motion.PROFILE.read_text(encoding="utf-8"))["fist"]
            ro.validate_channel(ro.interface_inventory(), self.hardware["hand_channel"], True)
            transport = motion.MotionTransport(self.hardware["hand_channel"])
            report = {}
            motion.run_motion(transport, target, report)
            self.target = report["target"]
            self.current = report["final_positions_raw"]
            self.updated = time.monotonic()
            self.ready = True
            while not self.event.is_set():
                state = transport.exchange(9, 0x02, .15)
                positions = transport.exchange(9, 0x09, .15)
                if not state or state[-1]["enable_state"] != 1 or not positions:
                    raise RuntimeError("Hand feedback/enable state lost")
                self.current = positions[-1]["positions_raw"]
                if max(abs(a-b) for a, b in zip(self.current, self.target)) > self.hardware["hand_position_tolerance_counts"]:
                    raise RuntimeError("Hand no longer holds approved closure target")
                transport.check_error()
                self.updated = time.monotonic()
                self.event.wait(.1)
        except Exception as exc:
            self.error, self.healthy, self.ready = str(exc), False, False
        finally:
            if transport:
                transport.close()

    def status(self):
        fresh = not self.real or not self.ready or time.monotonic()-self.updated < self.hardware["hand_timeout_s"]
        return self.ready and fresh, self.healthy and fresh

    def close(self):
        self.event.set()
        if self.thread:
            self.thread.join(timeout=2.)
