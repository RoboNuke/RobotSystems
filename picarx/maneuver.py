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
    global px, power

    off = -1 if left else 1
    px.backward(power)
    px.set_dir_servo_angle(off * 30)
    time.sleep(0.25)
    for i in range(30*off,-30*off,-1*off):
        px.set_dir_servo_angle(i)
        time.sleep(0.02)
    time.sleep(0.05)
    px.stop()
    

def kTurn(left):
    pass


def maneuverMenu():
    print("Maneuver Menu:")
    print("Forward@angle: f angle")
    print("Backward@angle: b angle")
    print("Parallel Park: p (l or r) i.e. to parallel park left: p l")

if __name__=="__main__":
    cmd = ""
    maneuverMenu()
    while True:
        cmd =  input("Select Maneuver:" )
        if cmd == "q":
            break

        try:
            man, arg = cmd.split()
        except:
            print("Invalid Command Please select from below menu:")
            maneuverMenu()
            continue
        

        if man == "f":
            forward(float(arg))

        if man == "b":
            backward(float(arg))

        if man == "p":
            parallel(arg == 'l')

