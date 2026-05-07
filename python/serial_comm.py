# ============================================================
#  serial_comm.py — Serial Communication Layer (with auto-reconnect)
# ============================================================

import serial
import threading
import queue
import time
import logging
from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT

logger = logging.getLogger(__name__)


class SerialComm:

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self.port          = port
        self.baud          = baud
        self.ser           = None
        self.rx_queue      = queue.Queue()
        self._running      = False
        self._thread       = None
        self._lock         = threading.Lock()
        self._disconnected = False   # flag set by reader thread on drop

    # ----------------------------------------------------------
    # CONNECTION
    # ----------------------------------------------------------

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port          = self.port,
                baudrate      = self.baud,
                timeout       = SERIAL_TIMEOUT,
                write_timeout = 1.0
            )
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self._running      = True
            self._disconnected = False
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()

            logger.info(f"Serial connected on {self.port} @ {self.baud} baud")
            return True

        except serial.SerialException as e:
            logger.error(f"Serial connect failed: {e}")
            return False

    def reconnect(self, retries: int = 10, delay: float = 1.5) -> bool:
        """Try to re-open the port after a disconnect."""
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        for attempt in range(1, retries + 1):
            time.sleep(delay)
            logger.info(f"Reconnect attempt {attempt}/{retries} ...")
            if self.connect():
                return True

        logger.error("Reconnect failed — giving up.")
        return False

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open and not self._disconnected

    def was_disconnected(self) -> bool:
        """Returns True once if a drop was detected (clears flag)."""
        if self._disconnected:
            self._disconnected = False
            return True
        return False

    # ----------------------------------------------------------
    # READER THREAD
    # ----------------------------------------------------------

    def _reader_loop(self):
        while self._running:
            try:
                if self.ser and self.ser.in_waiting:
                    raw  = self.ser.readline()
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.rx_queue.put(line)
                        logger.debug(f"RX ← {line}")
                else:
                    time.sleep(0.005)
            except serial.SerialException:
                logger.warning("Serial dropped — flagging for reconnect.")
                self._disconnected = True
                self._running = False
                break
            except Exception as e:
                logger.error(f"Reader error: {e}")

    # ----------------------------------------------------------
    # SEND
    # ----------------------------------------------------------

    def send(self, message: str):
        if not self.is_connected():
            return
        try:
            payload = (message.strip() + "\n").encode("utf-8")
            with self._lock:
                self.ser.write(payload)
            logger.debug(f"TX → {message}")
        except serial.SerialException:
            logger.warning("Send failed — serial dropped.")
            self._disconnected = True

    def send_servo(self, pan: int, tilt: int):
        pan  = max(0,  min(180, int(pan)))
        tilt = max(10, min(170, int(tilt)))
        self.send(f"X:{pan},Y:{tilt}")

    # ----------------------------------------------------------
    # RECEIVE
    # ----------------------------------------------------------

    def get_line(self) -> str | None:
        try:
            return self.rx_queue.get_nowait()
        except queue.Empty:
            return None

    def flush_rx(self):
        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break


# ----------------------------------------------------------
# PARSER HELPERS
# ----------------------------------------------------------

def parse_detect(line: str) -> dict | None:
    try:
        payload = line.split("DETECT:")[1]
        parts   = payload.split(",")
        return {"pan": int(parts[0]), "tilt": int(parts[1]), "dist": float(parts[2])}
    except (IndexError, ValueError):
        return None

def parse_aligned(line: str) -> dict | None:
    try:
        payload = line.split("ALIGNED:")[1]
        parts   = payload.split(",")
        return {"pan": int(parts[0]), "tilt": int(parts[1]), "dist": float(parts[2])}
    except (IndexError, ValueError):
        return None

def parse_state(line: str) -> str | None:
    try:
        return line.split("STATE:")[1].strip()
    except IndexError:
        return None
