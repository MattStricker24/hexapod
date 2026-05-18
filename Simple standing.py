import time
from servo import servo2040
from servo import ServoCluster

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

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_channel(ch, angle):
    servos.value(ch, clamp(angle, -90.0, 90.0), load=False)

def stand_up():
    for leg in range(1, 7):
        for joint in ["waist", "hip", "knee"]:
            ch = LEG_CHANNELS[leg][joint]
            angle = OFFSETS[leg][joint]
            set_channel(ch, angle)
    servos.load()

stand_up()