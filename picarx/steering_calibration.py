from picarx_improved import Picarx
import time
import math

if __name__=="__main__":
    px = Picarx()
    px.set_dir_servo_angle(0.0)
    px.forward(25)
    time.sleep(2)
    px.stop()

    dist = float(input("Distance of horizontal offset:"))
    length = float(input("Total distance forward:"))
    r = math.sqrt(dist**2 + length**2)
    offset_angle = math.degrees(math.atan2(math.fabs(dist)/r, length/r))

    offset_angle *= 1 if dist > 0 else -1

    print(f"Steering is off by an angle of {offset_angle}")

    px.dir_servo_calibrate(offset_angle)

    print("Calibration Set, have a good day!")
