# Trinket IO demo
# Welcome to CircuitPython 3.1.1 :)

import board
from digitalio import DigitalInOut, Direction, Pull
import adafruit_dotstar as dotstar
import time
import neopixel
import usb_cdc
import gc

# One pixel connected internally!
dot = dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1, brightness=0.2)

# Built in red LED
led = DigitalInOut(board.D13)
led.direction = Direction.OUTPUT

# NeoPixel strip (of 16 LEDs) connected on D4
NUMPIXELS = 21
neopixels = neopixel.NeoPixel(board.D4, NUMPIXELS, brightness=0.2, auto_write=False)


serial = usb_cdc.data
serial.timeout = 0
######################### HELPERS ##############################

# Helper to give us a nice color swirl
def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if (pos < 0):
        return (0, 0, 0)
    if (pos > 255):
        return (0, 0, 0)
    if (pos < 85):
        return (int(pos * 3), int(255 - (pos*3)), 0)
    elif (pos < 170):
        pos -= 85
        return (int(255 - pos*3), 0, int(pos*3))
    else:
        pos -= 170
        return (0, int(pos*3), int(255 - pos*3))

#swirl, but only between red and yellow to indicate wait
def waitPulse(pos):
    if (pos < 0):
        return (0, 0, 0)
    if (pos > 255):
        return (0, 0, 0)
    if (pos < 128):
        return (255, int(pos*2), 0)
    else:
        pos -= 128
        return (255, int(255 - pos*2), 0)


######################## AVAILABLE SERIAL COMMANDS ###############
# PODn - Sets this pod's number to n
# IDLE - Sets the pod state to idle
# CAL_STARTED - Sets the pod state to "Calibration Started"
#
##################################################################


######################### MAIN LOOP ##############################
Segments = {}
Segments["Ready"] = (1,2,3,4,6,8,9,10,11,12,16,19)
Segments["Error"] = (2,5,9,12,14,16,18,19,20)
Segments["Calibrating"] = (4,5,6,7,8,9,10,12,13,14,18,19,20)
Segments["Recording"] = (0,4,5,6,7,9,11,12,13,16,19)
Segments["Running"] = (2,3,5,10,11,12,16,19)
Segments["Background"] = (7,8,9,10,11,13,16,17,18,19,20)
Segments["POD1"] = (2,8,10,16)
Segments["POD2"] = (2,7,8,9,11,12,16)
Segments["POD3"] = (2,7,8,9,10,11,16)
Segments["POD4"] = (2,8,9,10,13,16)
Segments["POD5"] = (2,7,9,10,11,13,16)
Segments["POD6"] = (2,7,9,10,11,12,13,16)
Segments["POD7"] = (2,7,8,10,16)
Segments["POD8"] = (2,7,8,9,10,11,12,13,16)
Segments["POD9"] = (2,7,8,9,10,13,16)
Segments["POD10"] = (0,1,3,4,5,6,8,10,16)
Segments["POD11"] = (2,8,10,12,13,16) #future-proofing :-)
Segments["POD12"] = (0,1,2,4,5,8,10,16)
Segments[0] = (0,1,3,4,5,6)
Segments[1] = (1,3)
Segments[2] = (0,1,2,4,5)
Segments[3] = (0,1,2,3,4)
Segments[4] = (1,2,3,6)
Segments[5] = (0,2,3,4,6)
Segments[6] = (0,2,3,4,5,6)
Segments[7] = (0,1,3)
Segments[8] = (0,1,2,3,4,5,6)
Segments[9] = (0,1,2,3,6)
Segments[10] = (0,1,3,4,5,6,8,10)
Red = (128,0,0)
Green = (0,128,0)
Blue = (0,0,128)
White = (128,128,128)
Yellow = (128,128,0)

PodNumber = 1
FirstBoot = True

State = {}
State["Idle"] = (Segments["POD"+str(PodNumber)], Green)
State["Starting Calibration"] = (Segments["Calibrating"], Yellow)
State["Starting Broadcast"] = (Segments["Running"], Yellow)
State["Starting BG Capture"] = (Segments["Background"], Yellow)
State["Calibration Capturing Video"] = (Segments["Recording"], Red)
State["Calibration Data Transferring"] = (Segments["Calibrating"], Blue)
State["Calibration Running Calib"] = (Segments["Calibrating"], Blue)
State["Calibration Successful"] = (Segments["Calibrating"], Green)
State["Calibration Failed"] = (Segments["Calibrating"], Red)
State["Broadcast Running"] = (Segments["Running"], Green)
State["Capturing BG Images"] = (Segments["Background"], Red)
State["Pod Rebooting"] = (Segments["Ready"], Red)
State["Broadcast Stopping"] = (Segments["POD"+str(PodNumber)], Red)

CurrentPodState = "Idle"
i = 0
BackgroundCaptureCount = 0
in_data = bytearray()
Word = "Ready"
while True:
    # spin internal LED around! autoshow is on 
    dot[0] = wheel(i & 255)

    # check for a state change on the serial line
    if serial.in_waiting > 0:
        FirstBoot = False
        line = str(serial.readline(), 'utf8')
        #print("Got: ["+line+"]")
        # Process the command
        if line.startswith("POD"):
            print("Got a POD command:["+line+"]")
            PodNumber = int(line[3:-1])
            print("Setting Pod number to " + str(PodNumber))
            #Update state array
            State["Idle"] = (Segments["POD"+str(PodNumber)], Green)
            State["Broadcast Stopping"] = (Segments["POD"+str(PodNumber)], Red)
        elif line.startswith("BGCount"):
            print("Got a BG count update:["+line+"]")
            BackgroundCaptureCount = int(line[7:-1].rstrip())
            print("Set BG count to "+str(BackgroundCaptureCount))
        elif line.startswith("STATE"):
            newState = line[5:-1].rstrip()
            print("Changing state to: ["+newState+"]")
            if newState in State:
                CurrentPodState = newState
        bytesW = serial.write(bytes(line, "utf8"))
        print("Wrote "+str(bytesW)+" to the serial port")
    else:
        if i == 255:
            print(".")
    # update the display
    Segment = State[CurrentPodState][0]
    if(CurrentPodState == "Capturing BG Images"):     #"bgN"
        Segment += Segments[BackgroundCaptureCount/10]
    spinColor = wheel(i)
    waitColor = waitPulse(i)
    for p in range(NUMPIXELS):
        if (FirstBoot or CurrentPodState == "Pod Rebooting"):
            neopixels[p] = spinColor
        else:              
            if (p in Segment):            
                if (CurrentPodState == "Calibration Data Transferring"):
                    neopixels[p] = spinColor
                elif(CurrentPodState == "Broadcast Stopping"):
                    neopixels[p] = waitColor
                else:
                    neopixels[p] = State[CurrentPodState][1]
            else:
                neopixels[p] = (0,0,0)
    neopixels.show()

    i = (i+1) % 256  # run from 0 to 255
    time.sleep(.01) # make bigger to slow down
    gc.collect()
