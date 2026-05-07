# ============================================================
#  main.py — The Final Fusion: CV Tracking + Auto Radar Sweep
# ============================================================

import cv2
import logging
import math
import sys
import time
import numpy as np

from config import (
    CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS,
    WINDOW_NAME,
    SERVO_PAN_HOME,  SERVO_TILT_HOME,
    SERVO_PAN_MIN,   SERVO_PAN_MAX,
    SERVO_TILT_MIN,  SERVO_TILT_MAX,
    CAM_ABOVE_SERVO, TARGET_DIST,
    CAM_FOV_H, CAM_FOV_V,
    LASER_TRIM_TILT, LASER_TRIM_PAN,
    INVERT_PAN, INVERT_TILT,
)
from serial_comm import SerialComm
from tracker import TargetTracker

logging.basicConfig(
    level   = logging.INFO,
    format  = "[%(asctime)s] %(levelname)-7s %(name)s — %(message)s",
    datefmt = "%H:%M:%S"
)
logger = logging.getLogger("main")

SMOOTH      = 0.55
SWEEP_SPEED = 3.0
SEND_EVERY_N = 2

def pixel_to_servo(cx: int, cy: int) -> tuple[int, int]:
    # Step 1: pixel → camera angle  (screen-Y is inverted, so negate)
    cam_h = (cx / FRAME_WIDTH  - 0.5) *  CAM_FOV_H
    cam_v = (cy / FRAME_HEIGHT - 0.5) * -CAM_FOV_V   # + = up in world

    # Step 2: angle → 3D point at TARGET_DIST (camera frame, Y-up)
    Xt = TARGET_DIST * math.tan(math.radians(cam_h))
    Yt = TARGET_DIST * math.tan(math.radians(cam_v))

    # Step 3: translate to laser/servo origin
    # Laser is CAM_ABOVE_SERVO cm BELOW camera → target appears higher from laser
    x_srv = Xt
    y_srv = Yt + CAM_ABOVE_SERVO

    # Step 4: servo angles
    raw_pan  = math.degrees(math.atan2(x_srv,  TARGET_DIST))
    raw_tilt = math.degrees(math.atan2(y_srv,  TARGET_DIST))

    if not INVERT_PAN:  raw_pan  = -raw_pan
    if     INVERT_TILT: raw_tilt = -raw_tilt

    pan  = SERVO_PAN_HOME  + raw_pan  + LASER_TRIM_PAN
    tilt = SERVO_TILT_HOME + raw_tilt + LASER_TRIM_TILT

    return (int(max(SERVO_PAN_MIN,  min(SERVO_PAN_MAX,  pan))),
            int(max(SERVO_TILT_MIN, min(SERVO_TILT_MAX, tilt))))

# Trail buffer for radar sweep effect
_radar_trail = []

def draw_radar_ui(frame, pan_angle, dist, hand_found=False):
    global _radar_trail

    H, W = frame.shape[:2]

    # ── الأبعاد ──────────────────────────────────────────────
    R      = 100                          # نصف قطر الرادار
    cx     = W - R - 15                  # مركز الرادار أفقياً
    cy     = H - 10                       # مركز الرادار عند أسفل الشاشة
    C      = (cx, cy)

    # ── خلفية شبه شفافة ─────────────────────────────────────
    overlay = frame.copy()
    cv2.ellipse(overlay, C, (R+8, R+8), 0, 180, 360, (0, 20, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    # ── دوائر المسافة (max = 200 cm) ────────────────────────
    for r_pct, label in [(0.25,"50"), (0.5,"100"), (0.75,"150"), (1.0,"200")]:
        r = int(R * r_pct)
        cv2.ellipse(frame, C, (r, r), 0, 180, 360, (0, 60, 0), 1)
        tx = cx - r - 2
        ty = cy - 4
        cv2.putText(frame, label, (tx, ty),
                    cv2.FONT_HERSHEY_PLAIN, 0.65, (0, 100, 0), 1)

    # ── خطوط الزوايا (0 / 45 / 90 / 135 / 180) ─────────────
    for deg in [0, 45, 90, 135, 180]:
        a = math.radians(180 - deg)
        ex = cx + int(R * math.cos(a))
        ey = cy - int(R * math.sin(a))
        cv2.line(frame, C, (ex, ey), (0, 45, 0), 1)
        # تسمية الزاوية
        lx = cx + int((R + 10) * math.cos(a)) - 8
        ly = cy - int((R + 10) * math.sin(a)) + 4
        cv2.putText(frame, str(deg), (lx, ly),
                    cv2.FONT_HERSHEY_PLAIN, 0.55, (0, 120, 0), 1)

    # ── الإطار الخارجي ───────────────────────────────────────
    cv2.ellipse(frame, C, (R, R), 0, 180, 360, (0, 200, 0), 2)
    cv2.line(frame, (cx - R, cy), (cx + R, cy), (0, 200, 0), 2)

    # ── Sweep trail + خط المسح ───────────────────────────────
    _radar_trail.append(pan_angle)
    if len(_radar_trail) > 25:
        _radar_trail.pop(0)

    n = len(_radar_trail)
    for i, a_deg in enumerate(_radar_trail):
        ratio = i / n                              # 0.0 → 1.0 (قديم → حديث)
        a_rad_t = math.radians(180 - a_deg)
        ex = cx + int(R * math.cos(a_rad_t))
        ey = cy - int(R * math.sin(a_rad_t))
        if hand_found:
            # ذيل أحمر يتلاشى
            intensity = int(220 * ratio)
            color = (0, 0, intensity)
        else:
            # ذيل أخضر يتلاشى
            intensity = int(200 * ratio)
            color = (0, intensity, 0)
        cv2.line(frame, C, (ex, ey), color, 1)

    # خط المسح الحالي
    a_rad = math.radians(180 - pan_angle)
    ex = cx + int(R * math.cos(a_rad))
    ey = cy - int(R * math.sin(a_rad))
    line_color = (0, 0, 255) if hand_found else (0, 255, 80)
    cv2.line(frame, C, (ex, ey), line_color, 2)

    # ── نقطة المركز ─────────────────────────────────────────
    cv2.circle(frame, C, 4, (0, 255, 0), -1)

    # ── شريط المعلومات السفلي ───────────────────────────────
    bar_h = 32
    # خلفية الشريط على كامل عرض الشاشة باستثناء منطقة الرادار
    info_w = W - (R * 2 + 30)
    cv2.rectangle(frame, (0, H - bar_h), (info_w, H), (0, 15, 0), -1)
    cv2.rectangle(frame, (0, H - bar_h), (info_w, H), (0, 100, 0), 1)

    # STATUS
    status_color = (0, 255, 80) if hand_found else (0, 160, 0)
    status_txt   = "TARGET LOCKED" if hand_found else "SCANNING..."
    cv2.putText(frame, status_txt,
                (10, H - bar_h + 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1)

    # PAN
    cv2.putText(frame, f"PAN  {pan_angle:>3}",
                (160, H - bar_h + 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

    # DIST
    dist_color = (0, 255, 80) if 0 < dist < 90 else (0, 120, 0)
    cv2.putText(frame, f"DIST {dist:>3}cm",
                (290, H - bar_h + 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, dist_color, 1)

    # خط فاصل عمودي بين المقاطع
    for vx in [150, 280, 420]:
        if vx < info_w:
            cv2.line(frame, (vx, H - bar_h + 4), (vx, H - 4), (0, 80, 0), 1)

def main():
    logger.info("=== INITIALIZING FINAL TRACKING SYSTEM ===")
    comm = SerialComm()
    serial_ok = comm.connect()
    if serial_ok:
        time.sleep(1)
        comm.send("LASER:ON")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    tracker        = TargetTracker()
    pan, tilt      = float(SERVO_PAN_HOME), float(SERVO_TILT_HOME)
    last_dist      = 0
    frame_num      = 0
    hand_was_found = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret: continue
            frame_num += 1

            # --- 0. Auto-reconnect لو الأردوينو اتفصل وتوصل تاني ---
            if comm.was_disconnected():
                logger.warning("Arduino disconnected — trying to reconnect...")
                serial_ok = False
                hand_was_found = False
            if not serial_ok and not comm.is_connected():
                if comm.reconnect(retries=3, delay=1.0):
                    serial_ok = True
                    comm.send("LASER:ON")
                    logger.info("Reconnected successfully.")

            # --- 1. استقبال بيانات الرادار ---
            if serial_ok:
                while True:
                    line = comm.get_line()
                    if line is None: break
                    if "RADAR:" in line:
                        try:
                            last_dist = int(line.split(",")[1])
                        except: pass

            # --- 2. معالجة الصورة (CV) ---
            result = tracker.update(frame)

            # --- 3. منطق البازر والتتبع ---
            if result.found:
                if not hand_was_found and serial_ok:
                    comm.send("BUZZER:ON")
                    logger.info("Hand detected → BUZZER ON")
                hand_was_found = True
                status = "TARGET ACQUIRED"

                target_pan, target_tilt = pixel_to_servo(result.cx, result.cy)
                pan  += (1 - SMOOTH) * (target_pan - pan)
                tilt += (1 - SMOOTH) * (target_tilt - tilt)

            else:
                if hand_was_found and serial_ok:
                    comm.send("BUZZER:OFF")
                    logger.info("Hand lost → BUZZER OFF")
                hand_was_found = False
                status = "WAITING FOR HAND..." if serial_ok else "NO ARDUINO — RECONNECTING..."

                pan  += (1 - SMOOTH) * (SERVO_PAN_HOME  - pan)
                tilt += (1 - SMOOTH) * (SERVO_TILT_HOME - tilt)

            # --- 4. إرسال الأوامر للأردوينو ---
            if serial_ok and (frame_num % SEND_EVERY_N == 0):
                comm.send(f"X:{int(pan)},Y:{int(tilt)}")

            # --- 5. الرسم والعرض (HUD) ---
            display = tracker.draw_overlay(frame.copy(), result, status, int(pan), int(tilt))
            draw_radar_ui(display, int(pan), last_dist, hand_was_found)
            
            cv2.imshow(WINDOW_NAME, display)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        if serial_ok:
            comm.send("LASER:OFF")
            comm.disconnect()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()