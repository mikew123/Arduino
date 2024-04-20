import time
import rp2
from machine import Pin
import machine


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, in_shiftdir=1, autopush=True, push_thresh=32)
#@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def dmxData():
    
    # wait for start of frame MARK
    label("waitMark")
    wait(0,pin,0)
    
    # verify MARK is low at least 155(31*5) clks (77 usec)
    set(x,31)
    label("A1")
    jmp(pin, "waitMark")
    jmp(x_dec, "A1") [3]
   
    # wait for end of MARK
    wait(1,pin,0)
    
    # Set frame marker(0x0F) upper bits of first 16 bit data word
    set(y,15)

    # Debug MARK det signal on pin
    set(pins,1) [20]
    set(pins,0) 
#    nop()
#    nop()
    
    # wait for byte start (1>0) and center into first bit    
    label("waitStart")
    wait(0,pin,0) [1]
    
    # Upper 8 bit frame/data flag to ISR
    in_(y,8) 
    
    #skip start bit(8 clks)
    nop() [7]
    
    # collect lower 8 bits to ISR
    set(x, 7) 
    label("dmxData")
    in_(pins,1)
    
    # Debug data clk signal on pin
#    set(pins,1)
#    set(pins,0)
    nop()
    nop()

    jmp(x_dec, "dmxData") [4]
    
    # Keep collecting data if STOP bits detected
    set(y,0) [8] # Set data marker for next upper bytes
    jmp(pin, "waitStart") [2]
    
    # start MARK detection for next frame
    jmp("waitMark")


# DMX pins
#dmxReN = machine.Pin(13, machine.Pin.OUT)
#dmxRx  = machine.Pin(14, machine.Pin.IN)
#dmxTx  = machine.Pin(15, machine.Pin.OUT)

#dmxReN.value(0) # RS485 RX mode
#dmxTx.value(0)

dmxDataSm = rp2.StateMachine(0, dmxData, jmp_pin=Pin(14), freq=2000000, \
                             in_base=Pin(14), set_base=Pin(15))

dmxDataSm.active(1)

addr = 193
R=0
G=0
B=0

dmxByte=0

while True:
    state = 0
    while(state == 0):
        if (dmxDataSm.rx_fifo()>0):
            d = dmxDataSm.get()
            f0 = (d&0x000000FF)>>0
            # dmx frame start detected
            if(f0==0x0F):
                dmxByte = 0
                det = False
                state = 1
        dmxByte +=2
     
    while(state == 1):    
        # if fixture addr found get RGB from FIFO data
        if(dmxByte >= addr):
            # get 3 bytes for RGB
            if(addr==dmxByte):
                #get R and G from byte
                R = (d&0x0000FF00)>>8
                G = (d&0xFF000000)>>24
                #get next word from FIFO
                if (dmxDataSm.rx_fifo()>0):
                    d = dmxDataSm.get()
                    B = (d&0x0000FF00)>>8
                    dmxByte +=2
            else:
                #only get R from byte
                R = (d&0xFF000000)>>24
                #get next word from FIFO
                if (dmxDataSm.rx_fifo()>0):
                    d = dmxDataSm.get()
                    G = (d&0x0000FF00)>>8
                    B = (d&0xFF000000)>>24
                    dmxByte +=2
            
            state = 2
            print(R,G,B)    
            
        if (dmxDataSm.rx_fifo()>0):
            d = dmxDataSm.get()
            dmxByte +=2
         


        
