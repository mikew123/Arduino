from machine import Pin, PWM
import time

# Maximum led PWM high percent 
redLedMaxPct = 75
grnLedMaxPct = 75
bluLedMaxPct = 75

redLed=PWM(Pin(0))
redLed.freq(200)    #Match WASH FX PWM frequency
grnLed=PWM(Pin(1))
grnLed.freq(200)
bluLed=PWM(Pin(2))
bluLed.freq(200)

#Duty cyle 0 to 255 (75%) max matches WASH FX Val=255
def rgbLedDuty255(r,g,b):
    redLed.duty_u16(int((r*(redLedMaxPct/100.0)*65535)/255))
    grnLed.duty_u16(int((g*(grnLedMaxPct/100.0)*65535)/255))
    bluLed.duty_u16(int((b*(bluLedMaxPct/100.0)*65535)/255))

while True:
    rgbLedDuty255(0,0,0)
    time.sleep(1)
    rgbLedDuty255(128,0,0)
    time.sleep(1)
    rgbLedDuty255(0,128,0)
    time.sleep(1)
    rgbLedDuty255(0,0,128)
    time.sleep(1)
    rgbLedDuty255(128,0,128)
    time.sleep(1)
    rgbLedDuty255(128,128,128)
    time.sleep(1)
    


