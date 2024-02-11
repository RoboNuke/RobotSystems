import logging
from logging import DEBUG, INFO
# set logging format
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")


from logdecorator import log_on_start, log_on_end, log_on_error


try:
    from robot_hat import ADC, Pin
except ModuleNotFoundError:
    from sim_robot_hat import Pin, ADC


from picarx_improved import Picarx
import rossros as rr
import time
from inputimeout import inputimeout
import math
#logging.getLogger().setLevel(logging.DEBUG)

class GrayscaleSensing:
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

    #@log_on_end(DEBUG, "Grayscale Data:{result}")
    def getGrayscaleData(self):
        adcValues = []
        adcValues.append(self.chnLeft.read() - self._reference[0])
        adcValues.append(self.chnMid.read() - self._reference[1])
        adcValues.append(self.chnRight.read() - self._reference[2])
        return adcValues
    
    @log_on_end(logging.DEBUG, "Grayscale Updated")
    def update(self):
        return self.getGrayscaleData()

class UltraSonicInerpreter():
    def __init__(self, minDist = 0.05):
        self.minDist = minDist

    def update(self, rawDist):
        if rawDist == -1:
            return True
        else:
            return rawDist > self.minDist

class Interpretater:
    def __init__(self, polarity = 1, sensitivity=None):
        # polarity: 1 means line is bright, 0 means dark
        self.sen = sensitivity
        self.pol = polarity
        self.safe = True

    #@log_on_end(DEBUG, "Filtered Readings:{result}")
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
                logging.log(logging.DEBUG, f"Filted reading is {filt} and should not be possible")
                return 0.0
        elif(left and not right): # see only left
            return -1.0
        elif(not left and right): # see only right
            return 1.0
        else:
            logging.log(logging.DEBUG, f"Filted reading is {filt} and should not be possible")
            return None

    @log_on_start(logging.DEBUG, "Interpretter Loop Started")
    def update(self, grayscaleReading):
        filt = self.filter(grayscaleReading)
        LS = self.interpLineState(filt)
        logging.log(logging.DEBUG, f"Line State:{LS}")
        return LS
    
class Controller:
    def __init__(self, picar, scaling= 1.0, maxTurn = 30, pxSpeed=35):
        self.scale = scaling
        self.max = maxTurn
        self.px = picar
        self.safe = True
        self.pxSpeed = pxSpeed

    @log_on_start(logging.DEBUG, "Steering Angle Updated")
    def updateSteeringAngle(self, lineState):
        logging.log(logging.DEBUG, f"Controller got LS: {lineState}")
        if lineState != None:
            ang =  lineState**3 * self.max * self.scale
            logging.log(logging.DEBUG, f"Set angle to {ang}")
            self.px.set_dir_servo_angle(ang)
        else:
            logging.log(logging.DEBUG, "Controller reads None as Linestate")

    def updateDriving(self, isClearAhead):
        if isClearAhead:
            self.px.forward(self.pxSpeed)
        else:
            self.px.stop()


logging.getLogger().setLevel(logging.INFO)
if __name__=="__main__":
    # some constants and determined through careful testing and not made up
    ref = [31.28, 37.29, 36.66] 
    speed = 35
    maxAngle = 30
    polarity = 1.0
    scaling = 1.0
    avoidObs = True
    updateFreq = 1.0/20.0
    usUpdateFreq = 1.0/4.0
    timeout = 3.0

    # define busses
    gsSensorBus = rr.Bus([0.0]*3, "Grayscale Raw Data Bus")
    lineStateBus = rr.Bus([0]*3, "Line State Bus")
    if avoidObs:
        usSensorBus = rr.Bus(False, "Ultrasonic raw sensor reading")
        isClearBus = rr.Bus(False, "Interped Ultrasonic Sensor")
    termBus = rr.Bus(0, "Termination Bus")


    # define worker classes
    px = Picarx()
    grayscale = GrayscaleSensing("A0", "A1", "A2",ref)
    interp = Interpretater(polarity=polarity)
    cont = Controller(px, scaling, maxAngle, speed)
    if avoidObs:
        ultraInterp = UltraSonicInerpreter()



    # Wrap the classes into consumer/producer classes
    readGS = rr.Producer(
        grayscale.update,
        gsSensorBus,
        updateFreq,
        termBus,
        "Read raw grayscale sensor data"
    )

    if  avoidObs:
        readUS = rr.Producer(
            px.ultrasonic.read,
            usSensorBus,
            usUpdateFreq,
            termBus,
            "Read raw ultrasonic sensor data"
        )

    interpLS = rr.ConsumerProducer(
        interp.update,
        gsSensorBus,
        lineStateBus,
        updateFreq,
        termBus,
        "Grayscale Data Interping Line State"
    )

    if avoidObs:
        interpUS = rr.ConsumerProducer(
            ultraInterp.update,
            usSensorBus,
            isClearBus,
            usUpdateFreq,
            termBus,
            "Ultrasonic sensor reading to is clear"
        )


    controlSA = rr.Consumer(
        cont.updateSteeringAngle,
        lineStateBus,
        updateFreq,
        termBus,
        "Line state to steering control"
    )

    if avoidObs:
        controlF = rr.Consumer(
            cont.updateDriving,
            isClearBus,
            usUpdateFreq,
            termBus,
            "Is clear to forward driving"
        )
    
    termTimer = rr.Timer(
        termBus,
        timeout,
        updateFreq,
        termBus,
        "Termination Timer"
    )

    if avoidObs:
        printList = (gsSensorBus, lineStateBus, usSensorBus, isClearBus, termBus)
    else:
        printList = (gsSensorBus, lineStateBus, termBus)

    printBuses = rr.Printer(
        printList,
        #(gsSensorBus, termBus),
        0.25,
        termBus,
        "Print raw and derived data",
        "Data bus readings: "
    )
    pc_list = [readGS, interpLS, controlSA, printBuses]
    if avoidObs:
        for x in (readGS, interpUS, controlF):
            pc_list.append(x)

    pc_list.append(termTimer)

    rr.runConcurrently(pc_list)