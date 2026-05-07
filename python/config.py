# ============================================================
#  config.py — Central Configuration File
#  All tunable parameters live here. Change values here only.
# ============================================================

# ----------------------------------------------------------
# SERIAL COMMUNICATION
# ----------------------------------------------------------
SERIAL_PORT      = "COM3"       # Change to your Arduino port (e.g. COM5, /dev/ttyUSB0)
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT   = 0.05         # seconds — non-blocking read timeout

# ----------------------------------------------------------
# CAMERA
# ----------------------------------------------------------
CAMERA_INDEX     = 0            # 0 = built-in webcam, 1 = first external
FRAME_WIDTH      = 640
FRAME_HEIGHT     = 480
FRAME_FPS        = 30

# ----------------------------------------------------------
# PID CONTROLLER  (tune these for your servo response)
# ----------------------------------------------------------
PID_PAN_KP       = 0.060        # Proportional gain for pan axis
PID_PAN_KI       = 0.001        # Integral gain for pan axis
PID_PAN_KD       = 0.012        # Derivative gain for pan axis

PID_TILT_KP      = 0.055
PID_TILT_KI      = 0.001
PID_TILT_KD      = 0.010

PID_OUTPUT_LIMIT = 15.0         # Max degrees correction per frame

# ----------------------------------------------------------
# SERVO ANGLE LIMITS
# ----------------------------------------------------------
SERVO_PAN_MIN    = 0
SERVO_PAN_MAX    = 180
SERVO_PAN_HOME   = 90

SERVO_TILT_MIN   = 10
SERVO_TILT_MAX   = 170
SERVO_TILT_HOME  = 90

# ----------------------------------------------------------
# TRACKING LOGIC
# ----------------------------------------------------------
CENTER_DEADBAND_PX   = 15       # pixels — ignore error smaller than this
LOCK_CONFIRM_FRAMES  = 5        # consecutive frames needed before sending CONFIRM
TARGET_LOST_FRAMES   = 30       # consecutive miss frames before declaring lost
MIN_CONTOUR_AREA     = 500      # pixels² — minimum area to consider as target
MAX_CONTOUR_AREA     = 50000    # pixels² — maximum area (filter noise/bg)
MOG2_HISTORY         = 200
MOG2_THRESHOLD       = 40
MOG2_DETECT_SHADOWS  = False

# ----------------------------------------------------------
# GUI / DISPLAY
# ----------------------------------------------------------
WINDOW_NAME          = "Automated Target Tracking System"
COLOR_CROSSHAIR      = (0,   255, 0)    # Green
COLOR_TARGET_BOX     = (0,   0,   255)  # Red
COLOR_TRACK_BOX      = (255, 165, 0)    # Orange
COLOR_INFO_TEXT      = (255, 255, 255)  # White
COLOR_WARNING_TEXT   = (0,   165, 255)  # Orange
COLOR_SCANNING_OVLY  = (200, 200, 0)    # Yellow

# ----------------------------------------------------------
# LASER / SERVO GEOMETRY  (tune these for your setup)
# ----------------------------------------------------------
CAM_ABOVE_SERVO  = 55.0   # cm — laser is this far BELOW the camera
TARGET_DIST      = 80.0   # cm — estimated distance to hand (adjust to real distance)
CAM_FOV_H        = 60.0   # degrees — camera horizontal field of view
CAM_FOV_V        = 45.0   # degrees — camera vertical field of view

# Fine-tune trim (degrees) — add/subtract to fix remaining small error after geometry
# Positive TILT_TRIM  → laser aims higher
# Negative TILT_TRIM  → laser aims lower
# Positive PAN_TRIM   → laser aims right
LASER_TRIM_TILT  =  0.0
LASER_TRIM_PAN   =  0.0

INVERT_PAN  = False
INVERT_TILT = True

# ----------------------------------------------------------
# PROTOCOL TAGS (must match Arduino code exactly)
# ----------------------------------------------------------
TAG_STATE   = "STATE:"
TAG_DETECT  = "DETECT:"
TAG_SCAN    = "SCAN:"
TAG_ALIGNED = "ALIGNED:"
TAG_ACK     = "ACK:"
TAG_LOST    = "LOST"
