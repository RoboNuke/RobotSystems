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

import concurrent.futures
from readerwriterlock import rwlock


class SimpleBus():
    def __init__(self, default=None):
        self.message = default
        self.lock = rwlock.RWLockWriteD()

    def write(self, newMessage):
        with self.lock.gen_wlock():
            self.message = newMessage

    def read(self):
        with self.lock.gen_rlock():
            msg =  self.message
            return msg
    


class GrayscaleSensingProducer:
    @log_on_end(DEBUG, "Grayscale Sensor Initialized")
    def __init__(self, pinLeft="A0", pinMid="A1", pinRight="A2", reference = [1000]*3):
        self.safe = True

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
    
    @log_on_end(DEBUG, "Grayscale Updated")
    def update(self, sensorBus, freq):
        while self.safe:
            sensorBus.write(self.getGrayscaleData())
            time.sleep(1/freq)

class InterpretationConsumerProducer:
    def __init__(self, polarity = 1, sensitivity=None):
        # polarity: 1 means line is bright, 0 means dark
        self.sen = sensitivity
        self.pol = polarity
        self.safe = True

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

    @log_on_start(DEBUG, "Interpretter Loop Started")
    def update(self, sensorBus, lineBus, freq):
        while self.safe:
            grayscaleReading = sensorBus.read()
            filt = self.filter(grayscaleReading)
            LS = self.interpLineState(filt)
            logging.log(logging.DEBUG, f"Line State:{LS}")
            lineBus.write(LS)
            time.sleep(1/freq)

class ControllerConsumer:
    def __init__(self, picar, scaling= 1.0, maxTurn = 30):
        self.scale = scaling
        self.max = maxTurn
        self.px = picar
        self.safe = True

    @log_on_start(DEBUG, "Controller Loop Started")
    def update(self, lineBus, freq):
        while self.safe:
            lineState = lineBus.read()
            logging.log(logging.DEBUG, f"Controller got LS: {lineState}")
            if lineState != None:
                ang =  lineState**3 * self.max * self.scale
                logging.log(logging.DEBUG, f"Set angle to {ang}")
                self.px.set_dir_servo_angle(ang)
                logging.log(logging.DEBUG, f"Set angle to {ang}")
            else:
                logging.log(logging.DEBUG, "Controller reads None as Linestate")
            time.sleep(1/freq)

if __name__=="__main__":
    #testSensorInterp()
    #testLineState()
    #refLearner()
    ref = [31.28, 37.29, 36.66] 

    sensorBus = SimpleBus([0.0, 0.0, 0.0])
    lineStateBus = SimpleBus(0.0)


    px = Picarx()
    grayscale = GrayscaleSensingProducer("A0", "A1", "A2",ref)
    interp = InterpretationConsumerProducer(polarity=1)
    cont = ControllerConsumer(px, 1.0, 30)

    freq = 1.0
    px.forward(35)
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        eSensor = executor.submit(grayscale.update, sensorBus, freq)
        eInterp = executor.submit(interp.update, sensorBus, lineStateBus, freq)
        eControl = executor.submit(cont.update, lineStateBus, freq)


    logging.log(DEBUG, "Line Following Ended")
