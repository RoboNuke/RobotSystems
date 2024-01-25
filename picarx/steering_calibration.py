from picarx_improved import Picarx
import time
import math

if __name__=="__main__":
    unhappy = True
    px = Picarx()
    px.dir_servo_calibrate(0.0)
    while unhappy:

        #dist = float(input("Distance of horizontal offset:"))
        #length = float(input("Total distance forward:"))
        #r = math.sqrt(dist**2 + length**2)
        #offset_angle = math.degrees(math.atan2(math.fabs(dist)/r, length/r))

        #offset_angle *= 1 if dist > 0 else -1
        offset_angle=float(input("Input offset angle: "))
        print(f"Steering is off by an angle of {offset_angle}")

        px.set_dir_servo_angle(offset_angle)
        px.forward(50)
        time.sleep(2)
        px.stop()

        unhappy = not 'y' == input("Are you happy? ")

    
    px.dir_servo_calibrate(offset_angle)

    print("Calibration Set, have a good day!")

