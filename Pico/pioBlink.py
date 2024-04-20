import time
import rp2
from machine import Pin

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink():
    wrap_target()
    set(pins, 1)
    set(pins, 0)
    wrap()

sm = rp2.StateMachine(0, blink, freq=20000000, set_base=Pin(25))

sm.active(1)