from machine import Pin
import time

red = 0
grn = 1
blu = 2

redLed = Pin(red, Pin.OUT)
redLed.value(0)
bluLed = Pin(blu, Pin.OUT)
bluLed.value(0)
grnLed = Pin(grn, Pin.OUT)
grnLed.value(0)

while True:
    redLed.value(1)
    time.sleep(1)
    redLed.value(0)
    bluLed.value(1)
    time.sleep(1)
    bluLed.value(0)
    grnLed.value(1)
    time.sleep(1)
    grnLed.value(0)
   
