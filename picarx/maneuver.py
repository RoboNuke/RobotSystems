from picarx_improved import Picarx
import time
import math

px = Picarx()
power = 50

def forward(angle):
    global px, power
    px.set_dir_servo_angle(angle)
    px.forward(power)
    time.sleep(1.0)
    px.stop()

def backward(angle):
    global px, power
    px.set_dir_servo_angle(angle)
    px.backward(power)
    time.sleep(1.0)
    px.stop()

def parallel(left):
    pass

def kTurn(left):
    pass

if __name__=="__main__":
    cmd = ""
    print("Maneuver Menu:")
    print("Forward@angle: f angle")
    print("Backward@angle: b angle")

    while True:
        cmd =  input("Select Maneuver:" )
        if cmd == "q":
            break
        man, arg = cmd.split()
        arg = float(arg)

        if man == "f":
            forward(arg)
        if man == "b":
            backward(arg)

