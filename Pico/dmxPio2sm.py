import time
import rp2
from machine import Pin

# Mark frame detect state machine
# 7 instructions
@rp2.asm_pio()
def dmxFdet():
#    wrap_target()
    label(0)
    # wait for MARK falling edge
    wait(1,pin,0)
    
    label("A0")
    wait(0,pin,0)
    set(x,31)
    
    label("A1")
    jmp(pin, "A0")
    jmp(x_dec, "A1") [5]
   
    wait(1,pin,0)
    # Signal data state machines
    irq(0)
#    wrap()
    jmp(0)

# Put DMX data into FIFO
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW,in_shiftdir=0, out_shiftdir=0)#, autopush=True, push_thresh=32)
def dmxData():
#    wrap_target()
    label(0)
    # Wait for Frame detect signal
    wait(1,irq,0) [2] # wait before clearing IRQ for other SM to detect IRQ
    irq(clear,0)
    # Set frame marker for upper 8 bits of 16 bit frame type word
    set(y,15)
    
    # wait for byte start (1>0) and center into first bit    
    label("A")
    wait(0,pin,0) [1]
    # Upper 8 bits to ISR
    in_(y,8) 
    
    #skip start bit 
    nop() [7]
    
    # collect 8 bits
    set(x, 7) 
    label("B")
    in_(pins,1)
    set(pins,1)
    set(pins,0)
    jmp(x_dec, "B") [4]
    
    # Keep collecting data if STOP bits detected
#    nop() [8]
    set(y,0) [8] # Set data marker for next bytes in frame
    jmp(pin, "A") [2]
    jmp(0)
    
#    wrap()
    jmp(0)

dmxFdetSm = rp2.StateMachine(0, dmxFdet, jmp_pin=Pin(14), freq=2000000, in_base=Pin(14))

dmxFrameDataSm = rp2.StateMachine(2, dmxData, jmp_pin=Pin(14),  \
                freq=2000000, in_base=Pin(14), set_base=Pin(15))#,\
#                out_shiftdir=rp2.PIO.SHIFT_LEFT, push_thresh=32)

dmxFdetSm.active(1)
dmxFrameDataSm.active(1)

#while True:
#    if (dmxFrameDataSm.rx_fifo()>0):
#        d = dmxFrameDataSm.get()
#        print(hex(d))