# ============================================================
#  pid_controller.py — Dual-Axis PID Controller
#  Calculates servo correction from pixel error.
# ============================================================

import time
from config import (
    PID_PAN_KP,  PID_PAN_KI,  PID_PAN_KD,
    PID_TILT_KP, PID_TILT_KI, PID_TILT_KD,
    PID_OUTPUT_LIMIT,
    CENTER_DEADBAND_PX
)


class PIDAxis:
    """Single-axis PID controller with anti-windup and deadband."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = PID_OUTPUT_LIMIT):
        self.kp           = kp
        self.ki           = ki
        self.kd           = kd
        self.output_limit = output_limit

        self._integral    = 0.0
        self._prev_error  = 0.0
        self._prev_time   = None

    def compute(self, error: float) -> float:
        """
        Feed pixel error → receive servo delta in degrees.
        Positive error  → target is to the right / below center.
        """
        now = time.monotonic()

        if self._prev_time is None:
            dt = 0.033           # assume ~30fps on first call
        else:
            dt = now - self._prev_time
            if dt <= 0:
                dt = 1e-6        # prevent division by zero

        self._prev_time = now

        # Apply deadband — small errors produce zero output
        if abs(error) < CENTER_DEADBAND_PX:
            self._prev_error = error
            return 0.0

        # PID terms
        p = self.kp * error

        self._integral += error * dt
        # Anti-windup: clamp integral contribution
        integral_limit = self.output_limit / max(self.ki, 1e-9)
        self._integral  = max(-integral_limit, min(integral_limit, self._integral))
        i = self.ki * self._integral

        derivative = (error - self._prev_error) / dt
        d = self.kd * derivative

        self._prev_error = error

        output = p + i + d
        # Clamp output
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None


class DualAxisPID:
    """
    Wraps two PIDAxis instances (pan + tilt) and maps
    pixel error → (delta_pan_degrees, delta_tilt_degrees).
    """

    def __init__(self):
        self.pan  = PIDAxis(PID_PAN_KP,  PID_PAN_KI,  PID_PAN_KD)
        self.tilt = PIDAxis(PID_TILT_KP, PID_TILT_KI, PID_TILT_KD)

    def compute(self, error_x: float, error_y: float) -> tuple[float, float]:
        """
        error_x: pixels, positive = target right of center
        error_y: pixels, positive = target below center
        """
        # ====================================================
        # لو الكاميرا/الليزر بيمشي عكس الهدف يمين وشمال، حط سالب هنا
        # ولو شغال صح، شيل السالب
        delta_pan  =  -self.pan.compute(error_x)   
        
        # ====================================================
        # لو الكاميرا/الليزر بيمشي عكس الهدف فوق وتحت، حط سالب هنا
        # ولو شغال صح، شيل السالب
        delta_tilt =  -self.tilt.compute(error_y)  

        return delta_pan, delta_tilt

    def reset(self):
        self.pan.reset()
        self.tilt.reset()

    def update_gains(self,
                     pan_kp=None,  pan_ki=None,  pan_kd=None,
                     tilt_kp=None, tilt_ki=None, tilt_kd=None):
        """Hot-update PID gains without restarting."""
        if pan_kp  is not None: self.pan.kp  = pan_kp
        if pan_ki  is not None: self.pan.ki  = pan_ki
        if pan_kd  is not None: self.pan.kd  = pan_kd
        if tilt_kp is not None: self.tilt.kp = tilt_kp
        if tilt_ki is not None: self.tilt.ki = tilt_ki
        if tilt_kd is not None: self.tilt.kd = tilt_kd