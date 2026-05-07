# ============================================================
#  tracker.py — Hand Tracking Engine (MediaPipe Tasks API)
#  Pipeline:
#    HandLandmarker → detect & track hand landmarks every frame
#    Palm centre    → used as servo target
# ============================================================

import os
import cv2
import numpy as np
import logging
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python.vision import HandLandmarksConnections

from config import (
    FRAME_WIDTH, FRAME_HEIGHT,
    COLOR_CROSSHAIR, COLOR_TARGET_BOX, COLOR_TRACK_BOX,
    COLOR_INFO_TEXT, COLOR_WARNING_TEXT, COLOR_SCANNING_OVLY,
    WINDOW_NAME, LOCK_CONFIRM_FRAMES, TARGET_LOST_FRAMES
)

logger = logging.getLogger(__name__)

# Model file path (same directory as this script)
_HERE       = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH  = os.path.join(_HERE, "hand_landmarker.task")

# Palm landmark indices to compute palm centre
PALM_INDICES = [0, 1, 5, 9, 13, 17]

# Colours (BGR)
HAND_DOT_COLOR  = (0, 255, 0)   # green joints
HAND_LINE_COLOR = (255, 0, 0)   # blue skeleton
PALM_DOT_COLOR  = (0, 0, 255)   # red palm centre

# Hand skeleton connections (21 landmarks, same as old solutions API)
_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),           # thumb
    (0,5),(5,6),(6,7),(7,8),           # index
    (0,9),(9,10),(10,11),(11,12),      # middle
    (0,13),(13,14),(14,15),(15,16),    # ring
    (0,17),(17,18),(18,19),(19,20),    # pinky
    (5,9),(9,13),(13,17),              # palm arch
]


class TargetTracker:
    """Hand-tracking with MediaPipe Tasks HandLandmarker."""

    class TrackResult:
        __slots__ = ("found", "cx", "cy", "bbox", "error_x", "error_y",
                     "confidence", "landmarks")

        def __init__(self):
            self.found      = False
            self.cx         = 0
            self.cy         = 0
            self.bbox       = None        # (x, y, w, h) pixels
            self.error_x    = 0.0
            self.error_y    = 0.0
            self.confidence = 0.0
            self.landmarks  = None        # list of (px, py) for all 21 points

    def __init__(self):
        if not os.path.exists(MODEL_PATH):
            raise FileNotFoundError(
                f"Model not found: {MODEL_PATH}\n"
                "Download: hand_landmarker.task from MediaPipe model hub."
            )

        self.frame_cx = FRAME_WIDTH  // 2
        self.frame_cy = FRAME_HEIGHT // 2

        base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        options = mp_vision.HandLandmarkerOptions(
            base_options           = base_options,
            running_mode           = mp_vision.RunningMode.VIDEO,
            num_hands              = 1,
            min_hand_detection_confidence = 0.6,
            min_hand_presence_confidence  = 0.5,
            min_tracking_confidence       = 0.5,
        )
        self._detector = mp_vision.HandLandmarker.create_from_options(options)

        self._tracking_active = False
        self._confirm_count   = 0
        self._miss_count      = 0
        self._timestamp_ms    = 0

        logger.info("MediaPipe HandLandmarker loaded (Tasks API).")

    # ----------------------------------------------------------
    # PUBLIC API
    # ----------------------------------------------------------

    def update(self, frame: np.ndarray) -> "TargetTracker.TrackResult":
        result = TargetTracker.TrackResult()
        h, w   = frame.shape[:2]

        # Build MediaPipe image and advance timestamp
        self._timestamp_ms += 33   # ~30 fps
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        detection = self._detector.detect_for_video(mp_image, self._timestamp_ms)

        if detection.hand_landmarks:
            lm_norm = detection.hand_landmarks[0]

            # Normalised → pixel coords
            lm_px = [(int(lm.x * w), int(lm.y * h)) for lm in lm_norm]

            # Palm centre
            palm_x = int(np.mean([lm_px[i][0] for i in PALM_INDICES]))
            palm_y = int(np.mean([lm_px[i][1] for i in PALM_INDICES]))

            # Bounding box
            xs = [p[0] for p in lm_px]
            ys = [p[1] for p in lm_px]
            bx, by = min(xs), min(ys)
            bw, bh = max(xs) - bx, max(ys) - by

            result.found      = True
            result.cx         = palm_x
            result.cy         = palm_y
            result.bbox       = (bx, by, bw, bh)
            result.error_x    = float(palm_x - self.frame_cx)
            result.error_y    = float(palm_y - self.frame_cy)
            result.confidence = 1.0
            result.landmarks  = lm_px

            self._tracking_active = True
            self._confirm_count   = min(self._confirm_count + 1,
                                        LOCK_CONFIRM_FRAMES + 10)
            self._miss_count      = 0
        else:
            self._miss_count += 1
            if self._miss_count == TARGET_LOST_FRAMES + 1:
                logger.info("Hand lost — scanning...")
                self._tracking_active = False
                self._confirm_count   = 0

        return result

    def reset(self):
        self._tracking_active = False
        self._confirm_count   = 0
        self._miss_count      = 0

    def is_tracking(self) -> bool:
        return self._tracking_active

    def confirmed_lock(self) -> bool:
        return self._confirm_count >= LOCK_CONFIRM_FRAMES

    # ----------------------------------------------------------
    # HUD RENDERING
    # ----------------------------------------------------------

    def draw_overlay(self, frame: np.ndarray,
                     result: "TargetTracker.TrackResult",
                     arduino_state: str,
                     current_pan: int,
                     current_tilt: int) -> np.ndarray:

        fh, fw = frame.shape[:2]
        cx, cy = fw // 2, fh // 2

        # Crosshair at frame centre
        cv2.line(frame, (cx - 25, cy), (cx + 25, cy), COLOR_CROSSHAIR, 1)
        cv2.line(frame, (cx, cy - 25), (cx, cy + 25), COLOR_CROSSHAIR, 1)
        cv2.circle(frame, (cx, cy), 45, COLOR_CROSSHAIR, 1)

        if result.found and result.landmarks:
            lm = result.landmarks

            # Skeleton (blue lines)
            for s, e in _CONNECTIONS:
                if s < len(lm) and e < len(lm):
                    cv2.line(frame, lm[s], lm[e], HAND_LINE_COLOR, 2)

            # Joint dots (green)
            for pt in lm:
                cv2.circle(frame, pt, 5, HAND_DOT_COLOR, -1)

            # Palm centre dot (red)
            palm_pt = (result.cx, result.cy)
            cv2.circle(frame, palm_pt, 10, PALM_DOT_COLOR, -1)

            # Error vector
            cv2.line(frame, (cx, cy), palm_pt, COLOR_WARNING_TEXT, 1)

            # Error text
            bx, by, bw, bh = result.bbox
            cv2.putText(frame,
                        f"Ex:{result.error_x:+.0f}  Ey:{result.error_y:+.0f} px",
                        (bx, max(by - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, HAND_DOT_COLOR, 1)

        # Status bar (top-left)
        lock_str = ("LOCKED" if self.confirmed_lock()
                    else f"{self._confirm_count}/{LOCK_CONFIRM_FRAMES}")
        cv_str   = "TRACKING HAND" if self._tracking_active else "DETECTING HAND..."

        for i, txt in enumerate([
            f"System : {arduino_state}",
            f"CV     : {cv_str}",
            f"Pan:{current_pan}deg  Tilt:{current_tilt}deg",
            f"Lock   : {lock_str}",
        ]):
            cv2.putText(frame, txt,
                        (10, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_INFO_TEXT, 1)

        # Bottom hint
        cv2.putText(frame,
                    "R=Reset | Q=Quit",
                    (10, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_SCANNING_OVLY, 1)

        return frame
