from cflib.positioning.motion_commander import MotionCommander
import time

DEFAULT_HEIGHT = 0.4

# Offboard control - Take off
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.up(0.3)
        time.sleep(3)
        mc.stop()