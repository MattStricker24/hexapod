import math
import time
from servo import servo2040
from servo import ServoCluster

k3 = 17.0
k2 = 7.9

pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
servos = ServoCluster(0, 0, pins)
servos.enable_all()
time.sleep(1)

LEG_CHANNELS = {
    1: {"waist": 0,  "hip": 1,  "knee": 2},
    2: {"waist": 3,  "hip": 4,  "knee": 5},
    3: {"waist": 6,  "hip": 7,  "knee": 8},
    4: {"waist": 9,  "hip": 10, "knee": 11},
    5: {"waist": 12, "hip": 13, "knee": 14},
    6: {"waist": 15, "hip": 16, "knee": 17},
}

OFFSETS = {
    1: {"waist": -7.0, "hip": -3.0, "knee":  6.0},
    2: {"waist":  0.0, "hip": -4.0, "knee": -6.0},
    3: {"waist": -7.0, "hip": -3.0, "knee": -5.0},
    4: {"waist":  2.0, "hip": -9.0, "knee":  5.0},
    5: {"waist": -9.0, "hip": -3.0, "knee":  3.0},
    6: {"waist":  4.0, "hip":  0.0, "knee":  9.0},
}

WAIST_MIN = -60.0
WAIST_MAX = 60.0
HIP_MIN = -90.0
HIP_MAX = 45.0
KNEE_MIN = 0.0
KNEE_MAX = 155.0

BASE_X = 0.0
BASE_Y = 6.0
BASE_Z = -15.0

TRIPOD_A = [1, 3, 5]
TRIPOD_B = [2, 4, 6]

A_PHASES = [
    (-1.0, 0.0, -2.0),
    ( 1.0, 0.0, -2.0),
    ( 1.0, 0.0,  0.0),
    (-1.0, 0.0,  0.0),
]

B_PHASES = [
    ( 1.0, 0.0,  0.0),
    (-1.0, 0.0,  0.0),
    (-1.0, 0.0, -2.0),
    ( 1.0, 0.0, -2.0),
]

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ik_leg(A, B, C):
    j1 = math.atan2(A, B)
    L = min(math.sqrt(A*A + B*B + C*C), k2 + k3)

    arg_j3 = (k2*k2 + k3*k3 - L*L) / (2.0 * k2 * k3)
    arg_j3 = clamp(arg_j3, -1.0, 1.0)
    j3 = math.acos(arg_j3)

    proj = math.sqrt(A*A + B*B)
    arg_b = (L*L + k2*k2 - k3*k3) / (2.0 * L * k2) if L != 0 else 1.0
    arg_b = clamp(arg_b, -1.0, 1.0)
    b = math.acos(arg_b)

    a = math.atan(C / -proj)
    j2 = b - a

    return j1, j2, j3, a, b, L

def move_tripod(tripod, dx, dy, dz):
    for leg in tripod:
        p = [0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3][leg - 1]

        D = dx * math.cos(p) - dy * math.sin(p)
        E = dx * math.sin(p) + dy * math.cos(p)

        x = BASE_X + D
        y = BASE_Y + E
        z = BASE_Z + dz

        j1, j2, j3, a, b, L = ik_leg(x, y, z)

        waist_deg = clamp(math.degrees(j1) + OFFSETS[leg]["waist"], WAIST_MIN, WAIST_MAX)
        hip_deg = clamp(math.degrees(j2) + OFFSETS[leg]["hip"], HIP_MIN, HIP_MAX)
        knee_deg = clamp((180 - math.degrees(j3)) + OFFSETS[leg]["knee"], KNEE_MIN, KNEE_MAX)

        servos.to_percent(LEG_CHANNELS[leg]["waist"], waist_deg, -90, 90, load=False)
        servos.to_percent(LEG_CHANNELS[leg]["hip"], hip_deg, -90, 90, load=False)
        servos.to_percent(LEG_CHANNELS[leg]["knee"], knee_deg, 0, 180, load=False)

    servos.load()

if __name__ == '__main__':
    index = 0
    while True:
        move_tripod(TRIPOD_A, *A_PHASES[index])
        move_tripod(TRIPOD_B, *B_PHASES[index])
        index = (index + 1) % 4
        time.sleep(1)

servos.disable_all()