import logging
from logging import DEBUG, INFO
# set logging format
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")


from logdecorator import log_on_start, log_on_end, log_on_error


try:
    from robot_hat import ADC
except ModuleNotFoundError:
    from sim_robot_hat import ADC


from picarx_improved import Picarx
import time
from inputimeout import inputimeout
import math
logging.getLogger().setLevel(logging.DEBUG)

class GrayscaleSensing:
    @log_on_end(DEBUG, "Grayscale Sensor Initialized")
    def __init__(self, pinLeft, pinMid, pinRight, reference = [1000]*3):
        
        if isinstance(pinLeft,str):
            self.chnLeft= ADC(pinLeft)
            self.chnMid = ADC(pinMid)
            self.chnRight = ADC(pinRight)
        else:
            self.chnLeft = ADC('A0')
            self.chnMid = ADC('A1')
            self.chnRight = ADC('A2')
            
        
        self.reference(reference)

    @log_on_start(DEBUG, "Setting Reference to: {ref}")
    def reference(self, ref):
        if isinstance(ref, int) or isinstance(ref, float):
            self._reference = [ref] * 3
        elif isinstance(ref, list) and len(ref) == 3:
            self._reference = ref
        else:
            raise TypeError("reference parameter must be \'int\', \'float\', or 1*3 list.")

    @log_on_end(INFO, "Grayscale Data:{result}")
    def getGrayscaleData(self):
        adcValues = []
        adcValues.append(self.chnLeft.read() - self._reference[0])
        adcValues.append(self.chnMid.read() - self._reference[1])
        adcValues.append(self.chnRight.read() - self._reference[2])
        return adcValues
    
    @log_on_end(DEBUG, "Grayscale Sensor Read:{result}")
    def read(self):
        return self.getGrayscaleData()

class Interpretation:
    def __init__(self, polarity = 1, sensitivity=None):
        # polarity: 1 means line is bright, 0 means dark
        self.sen = sensitivity
        self.pol = polarity

    @log_on_end(DEBUG, "Filtered Readings:{result}")
    @log_on_start(DEBUG, "Filtering {rawReading}")
    def filter(self, rawReading):
        # returns 1 if can see line and 0 if can't for reach element of rawReading
        avg = sum(rawReading)/len(rawReading)
        logging.log(DEBUG, f"Filter Avg: {math.fabs(avg)}")
        maxDiff = max(rawReading) - min(rawReading)
        if maxDiff < 10:
            if math.fabs(avg) < 15.0:
                return [0,0,0]
            return [1,1,1]
        adj = [(x - avg) if self.pol else (avg - x) for x in rawReading]
        logging.log(DEBUG, f"Filter Adjusted:{adj}")
        filtered = [1 if x > 0 else 0 for x in adj]

        return filtered

    @log_on_end(DEBUG, "Interpreted Line State: {result}")
    def interpLineState(self, filt):
        # there are 2^3 possible outputs one of which is impossible [1,1,1]
        # meaning 7 total possibilities
        left, mid, right = filt
        if(sum(filt) == 0): # can't see the line
            return None
        elif(mid): # we can see the line in the middle sensor
            if(left and not right): # see mid + left
                return -0.5
            elif(right and not left): # see mid + right
                return 0.5
            elif(not right and not left): # see only mid
                return 0.0
            else: # see all three
                logging.log(logging.WARN, f"Filted reading is {filt} and should not be possible")
                return None
        elif(left and not right): # see only left
            return -1.0
        elif(not left and right): # see only right
            return 1.0
        else:
            logging.log(logging.WARN, f"Filted reading is {filt} and should not be possible")
            return None

    @log_on_start(DEBUG, "Calc Line State raw Reading: {grayscaleReading}")
    @log_on_end(DEBUG, "Calculated Line State: {result}")
    def calcLineState(self, grayscaleReading):
        filt = self.filter(grayscaleReading)
        LS = self.interpLineState(filt)
        return LS

class Controller:
    def __init__(self, scaling= 1.0, maxTurn = 30):
        self.scale = scaling
        self.max = maxTurn

    @log_on_start(DEBUG, "Recieved Line State:{lineState}")
    @log_on_end(DEBUG, "Calc Steering Angle: {result}")
    def getSteeringAngle(self, lineState):
        return lineState**3 * self.max * self.scale


def testSensorInterp():
    grayscale = GrayscaleSensing("A0", "A1", "A2")
    interp = Interpretation(polarity=0)
    for i in range(3):
        raw = grayscale.read()
        raw[0] *=2
        raw[2] /=2
        filt = interp.filter(raw)
        #time.sleep(2.0)

def testLineState():
    interp = Interpretation(polarity=0)
    for l in [0,1]:
        for m in [0,1]:
            for r in [0,1]:
                fil = [l, m, r]
                print(f"{fil}:{interp.interpLineState(fil)}")

def refLearner():
    px = Picarx()
    grayscale = GrayscaleSensing("A0", "A1", "A2")
    px.set_dir_servo_angle(0.0)
    px.forward(40)
    avg = [0,0,0]
    for i in range(100):
        raw = grayscale.read()
        for i in range(3):
            avg[i] += raw[i]
        time.sleep(0.05)
    print(f"Average Reading: {avg[0]/100}, {avg[1]/100}, {avg[2]/100}")
if __name__=="__main__":
    #testSensorInterp()
    #testLineState()
    #refLearner()
    #grayscale = GrayscaleSensing("A0", "A1", "A2")
    #px = Picarx()
    #px.stop()
    #while True:
    #    print(grayscale.read())
    #    time.sleep(1.0)
    ref = [-992.82, -990.82, -990.72] 

    px = Picarx()
    grayscale = GrayscaleSensing("A0", "A1", "A2",ref)
    interp = Interpretation(polarity=1)
    cont = Controller(1.0, 30)

    safe = True
    while safe:
        try:
            #time_over = inputimeout(prompt="\b", timeout=0.025)
            true + "test"
            safe = False
            logging.log(DEBUG, "Safe set to False in Line Following")
            break
        except:
            safe = True
        time.sleep(0.025)
        
        raw = grayscale.read()
        ls = interp.calcLineState(raw)
        if ls == None:
            px.stop()
            continue
        px.forward(35)
        angle = cont.getSteeringAngle(ls)
        px.set_dir_servo_angle(angle)
    logging.log(DEBUG, "Line Following Ended")

        



