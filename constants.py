### ===================================== ###
# TUNING CONSTANTS FOR PARTICLE FILTERING

# Normal distribution of the distance reweighting max error in centimetres
MAX_LANDMARK_ERROR = 40

# When moving what percentage error is allowed
MOTOR_DRIFT_MAX = 0.005
ANGLE_ERROR_MAX = 0.01
LINEAR_ERROR_MAX = 0.02

# When propogating what is the maximum error allowed in centimetres/radians
LINEAR_PROP_MAX = 12
ANGULAR_PROP_MAX = 0.1

# Propogation numbers for the particles
INIT_PARTICLES = 5000
TRIMMED_PARTICLES = 2000
RESAMPLED_PARTICLES = 3000

TRIM_THRESHOLD = 0.7

# Map array dimensions
MAP_HEIGHT = 601
MAP_WIDTH = 601

### ===================================== ###
# TUNING CONSTANTS FOR MOVEMENT

# Rotation
ROTATE_P = 0.02 
ROTATE_I = 0.00003

# Linear movement
LINEAR_P = 0.012
LINEAR_I = 0.000002

LINEAR_ACC_CAP = 0.03
LINEAR_VEL_CAP = 0.3

SKEW_P = 0.1

### ===================================== ###
# TUNING CONSTANTS FOR CORNER DETECTION

# For detecting corners on a global map
GLOBAL_CORNER_SIZE = 10
GLOBAL_SOBEL = 29
GLOBAL_K = 0.06

# For detecting corners in a single field of view
LOCAL_CORNER_SIZE = 10
LOCAL_SOBEL = 27
LOCAL_K = 0.06

THRESH_HARRIS = 90 #90

