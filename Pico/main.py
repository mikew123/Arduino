import machine
from machine import Pin, PWM
import utime
import time
import ustruct
import sys
import rp2
import machine


###############################################################################
# Settings


###############################################################################
# Functions

# reverse bit order for LSB first
# NOTE: needed since LSB mode is not implimented in SPI driver
def revBits(byte):
    rbyte = 0x00
    rbyte |= ((byte&0x80)>>7)
    rbyte |= ((byte&0x40)>>5)
    rbyte |= ((byte&0x20)>>3)
    rbyte |= ((byte&0x10)>>1)
    rbyte |= ((byte&0x08)<<1)
    rbyte |= ((byte&0x04)<<3)
    rbyte |= ((byte&0x02)<<5)
    rbyte |= ((byte&0x01)<<7)
    return(rbyte)
    
# SPI display and button functions

# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(17, machine.Pin.OUT)

# Initialize SPI0 200 KHz to match WASH FX display driver
spi = machine.SPI(0,
                  baudrate=200000,
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(18),
                  mosi=machine.Pin(19),
                  miso=machine.Pin(16))


# Command 1
# Modes 10= 6 grids, 12 segments
#       11= 7 grids, 11 segments
def setDisplayMode(mode):
    msg = bytearray()
    msg.append(revBits(0x00 | (mode & 0x03)))
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    time.sleep(0.0001)

# Command 2
def setDataRdWr(rd):
    msg = bytearray()
    msg.append(revBits(0x40 | ((rd<<1) & 0x02)))
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    time.sleep(0.0001)

#Command 4
def setDisplayControl(on, pwm):
    msg = bytearray()
    msg.append(revBits(0x80 | ((on<<3) & 0x08) | (pwm & 0x07)))
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    time.sleep(0.0001)

# read buttons
def readButtonsX4():
    msg = bytearray()
    msg.append(revBits(0x42)) # rd=1, inc addr
    cs.value(0)
    spi.write(msg)
    time.sleep(0.0001)
    msg = spi.read(5, 0xFF)
    cs.value(1)
    time.sleep(0.0001)
    buttons  = (msg[1]&0x80) >> 6
    buttons |= (msg[1]&0x10) >> 4
    buttons |= (msg[0]&0x80) >> 4
    buttons |= (msg[0]&0x10) >> 2
    return(buttons)
# clear display RAM
def setDisplayClr():
    msg = bytearray()
    msg.append(revBits(0xC0)) # Command 3 start address at 0
    msg.append(0)    #00
    msg.append(0)    #01
    msg.append(0)    #02
    msg.append(0)    #03
    msg.append(0)    #04
    msg.append(0)    #05
    msg.append(0)    #06
    msg.append(0)    #07
    msg.append(0)    #08
    msg.append(0)    #09
    msg.append(0)    #0A
    msg.append(0)    #0B
    msg.append(0)    #0C
    msg.append(0)    #0D
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    time.sleep(0.0001)

# set 7-segment display 4 digits
# each digit has a 12 bit (3 nibble) segment enable
def setDisplayX4(d0, d1, d2, d3):
    msg = bytearray()
    msg.append(revBits(0xC0)) # Command 3 start address at 0
    msg.append(revBits(d0 & 0xFF))      #00
    msg.append(revBits((d0>>8) & 0x0F)) #01
    msg.append(revBits(d1 & 0xFF))      #02
    msg.append(revBits((d1>>8) & 0x0F)) #03
    msg.append(revBits(d2 & 0xFF))      #04
    msg.append(revBits((d2>>8) & 0x0F)) #05
    msg.append(revBits(d3 & 0xFF))      #06
    msg.append(revBits((d3>>8) & 0x0F)) #07
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    time.sleep(0.0001)
    
def clearDisplayRam():
    setDataRdWr(0)         # wr
    setDisplayClr()        # Clear RAM
    setDisplayMode(3)      #7 grids, 11 segments
    setDisplayControl(1,7) # display ON, max

def writeDisplayDigitsX4(d0, d1, d2, d3):
    setDataRdWr(0)         # wr
    setDisplayX4(d0,d1,d2,d3)
    setDisplayMode(3)      #7 grids, 11 segments
    setDisplayControl(1,7) # display ON, max
    time.sleep(0.001)
    

char2DisplaySegs = {
          #76543210
    " ": 0b00000000,
    "0": 0b11111010,
    "1": 0b00100010,
    "2": 0b10111001,
    "3": 0b10101011,
    "4": 0b01100011,
    "5": 0b11001011,
    "6": 0b11011011,
    "7": 0b10100010,
          #76543210
    "8": 0b11111011,
    "9": 0b11101011,
    "A": 0b11110011,
    "B": 0b01011011,
    "C": 0b11011000,
    "D": 0b00111011,
    "E": 0b11011001,
    "F": 0b11010001,
          #76543210
    "G": 0b11011010,
    "H": 0b01110011,
    "I": 0b01010000,
    "J": 0b00101010,
    "K": 0b11010011,
    "L": 0b01011000,
    "M": 0b10010011,
    "N": 0b00010011,
          #76543210
    "O": 0b00011011,
    "P": 0b11111001,
    "Q": 0b11100011,
    "R": 0b00010001,
    "S": 0b11001010,
    "T": 0b11010000,
    "U": 0b01111010,
    "V": 0b01101010,
          #76543210
    "W": 0b01101000,
    "X": 0b10001001,
    "Y": 0b01101011,
    "Z": 0b10111000
          #76543210
    }
    
def displayStringX4(s):
    chars = [char for char in s]
    d0 = int(char2DisplaySegs[chars[0]])
    d1 = int(char2DisplaySegs[chars[1]])
    d2 = int(char2DisplaySegs[chars[2]])
    d3 = int(char2DisplaySegs[chars[3]])
    writeDisplayDigitsX4(d3,d1,d2,d0)
    
# set 4 digit display 1 char and 3 digits leading zeros
def setDisplay(c,n):
    s = "{:1s}{:03d}".format(c,n)
    displayStringX4(s)
    

##################################
# LED functions

# Maximum led PWM high percent 
redLedMaxPct = 75
grnLedMaxPct = 75
bluLedMaxPct = 75

redPin = 0
bluPin = 1
grnPin = 2

pwmFreq = 200 #Match WASH FX PWM frequency

redLed=PWM(Pin(redPin))
redLed.freq(pwmFreq)    
grnLed=PWM(Pin(bluPin))
grnLed.freq(pwmFreq)
bluLed=PWM(Pin(grnPin))
bluLed.freq(pwmFreq)

#Duty cyle 0 to 255 (75%) max matches WASH FX Val=255
def rgbLedDuty255(r,g,b):
    redLed.duty_u16(int((r*(redLedMaxPct/100.0)*65535)/255))
    grnLed.duty_u16(int((g*(grnLedMaxPct/100.0)*65535)/255))
    bluLed.duty_u16(int((b*(bluLedMaxPct/100.0)*65535)/255))


##################################
# DMX functions

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, fifo_join=rp2.PIO.JOIN_RX, \
             in_shiftdir=1, autopush=True, push_thresh=32)
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
    set(y,0x0F)
    #align to first byte of ISR
    mov(isr,null)

    # Debug MARK det signal on pin
#    set(pins,1) [20]
#    set(pins,0) 
    nop()
    nop()
    
    # wait for byte start (1>0) and center into first bit    
    label("waitStart")
    wait(0,pin,0) [1]
    
    # 8 bit frame/data flag F/0 to ISR
    in_(y,8) 
    
    #skip start bit(8 clks)
    nop() [7]
    
    # collect 8 bits of data in ISR
    set(x, 7) 
    label("dmxData")
    in_(pins,1)
    
    # Debug data clk signal on pin
    set(pins,1)
    set(pins,0)
#    nop()
#    nop()

    jmp(x_dec, "dmxData") [4]
    
    # Set data marker for data flag=0 bytes
    set(y,0) [8]
    
    # Keep collecting data if STOP(HIGH) bit detected
    jmp(pin, "waitStart") [2]
    
    # start MARK detection for next frame
    jmp("waitMark")

# DMX pins
dmxReN = machine.Pin(13, machine.Pin.OUT)
dmxRx  = machine.Pin(14, machine.Pin.IN)
dmxTx  = machine.Pin(15, machine.Pin.OUT)

dmxReN.value(0) # RS485 RX mode
dmxTx.value(0)

dmxDataSm = rp2.StateMachine(0, dmxData, jmp_pin=Pin(14), freq=2000000, \
                             in_base=Pin(14), set_base=Pin(15))

dmxDataSm.active(1)

##############################################################
# Runtime code

debug = Pin(12, Pin.OUT)

R=0
G=0
B=0
dmxByte=0

clearDisplayRam()

prevButtons = 0b0000

# these variables need to be non-volatile and restored at reset
try:
    with open("washFxNonVolatileData.csv","r") as f:
        # If open is succesfull then restore data from file
        data = f.read().strip().split()
        print(data)
        
        dstate  = data[0].replace(",","")
        lstate  = data[1].replace(",","")
        addrVal = int(data[2].replace(",",""))
        redVal  = int(data[3].replace(",",""))
        grnVal  = int(data[4].replace(",",""))
        bluVal  = int(data[5].replace(",",""))
        lAddrVal=addrVal
        lRedVal=redVal
        lGrnVal=grnVal
        lBluVal=bluVal
        print(lstate, dstate, addrVal,redVal,grnVal,bluVal)
        
        if(lstate == "off"):     
            rgbLedDuty255(0,0,0)
        elif(lstate == "addr"):     
            rgbLedDuty255(0,0,0)
        elif((lstate == "red")or(lstate == "grn")or(lstate == "blu")):     
            rgbLedDuty255(lRedVal,lGrnVal,lBluVal)
      
except:
    print("washFxNonVolatileData.csv file not present - use defaults")
    dstate = "off" # display state
    lstate = "off" # led state
    addrVal = 1
    redVal = 0
    grnVal = 0
    bluVal = 0
    lAddrVal=addrVal
    lRedVal=redVal
    lGrnVal=grnVal
    lBluVal=bluVal
    
    rgbLedDuty255(0,0,0)

mainStart = True
buttons = 0
# main exec loop
while True:
    buttons = readButtonsX4()
    if((buttons != prevButtons) or mainStart):
        mainStart = False
        # run code once when button pushed
        if(prevButtons == 0b0000):
            menu  = (buttons == 0b0001)
            up    = (buttons == 0b0010)
            dn    = (buttons == 0b0100)
            enter = (buttons == 0b1000)
           
            if (menu != 0):
                if(dstate == "off"):
                    dstate = "addr"
                elif(dstate == "addr"):
                    dstate = "red"              
                elif(dstate == "red"):
                    dstate = "grn"      
                elif(dstate == "grn"):
                    dstate = "blu"
                elif(dstate == "blu"):
                    dstate = "off"
           
            if(dstate == "addr"):
                if(up!=0): addrVal += 1
                if(dn!=0): addrVal -= 1
                if(addrVal > 512): addrVal = 1
                if(addrVal < 1):   addrVal = 512
                setDisplay("A",addrVal)
                    
            elif(dstate == "red"):
                if(up!=0): redVal += 1
                if(dn!=0): redVal -= 1
                if(redVal > 255): redVal = 0
                if(redVal < 0):   redVal = 255
                setDisplay("R",redVal)
                    
            elif(dstate == "grn"):
                if(up!=0): grnVal += 1
                if(dn!=0): grnVal -= 1
                if(grnVal > 255): grnVal = 0
                if(grnVal < 0):   grnVal = 255
                setDisplay("G",grnVal)
                    
            elif(dstate == "blu"):
                if(up!=0): bluVal += 1
                if(dn!=0): bluVal -= 1
                if(bluVal > 255): bluVal = 0
                if(bluVal < 0):   bluVal = 255
                setDisplay("B",bluVal)
                    
            elif(dstate == "off"):
                    clearDisplayRam()
            
            # When Enter pushed change LED state to display state
            if(enter != 0):
                lstate = dstate
                lAddrVal=addrVal
                lRedVal=redVal
                lGrnVal=grnVal
                lBluVal=bluVal
                if(lstate == "off"):
                    rgbLedDuty255(0,0,0)
                    print("LED and display OFF")
                elif(lstate == "addr"):
                    rgbLedDuty255(0,0,0)
                    print("LED ON, DMX mode")
                elif((lstate == "red")or(lstate == "grn")or(lstate == "blu")):
                    rgbLedDuty255(lRedVal,lGrnVal,lBluVal)
                    print("LED ON, RGB mode")
                    
            # save states and values in Flash file
            if((enter != 0)or(menu != 0)):
                with open("washFxNonVolatileData.csv","w") as f:
                    csvData = dstate+", " \
                    +lstate+", "          \
                    +str(lAddrVal)+", "    \
                    +str(lRedVal)+", "     \
                    +str(lGrnVal)+", "     \
                    +str(lBluVal)          
                    f.write(csvData)
                    print(csvData)
                    

        prevButtons = buttons
        # end run code once when button pushed

    # Get DMX RGB data while in addr mode
    if (lstate=="addr"):
        DmxState=0
        while(DmxState == 0):
           if (dmxDataSm.rx_fifo()):
#                debug.toggle()
                d = dmxDataSm.get()
                if((d&0xFF)==0x0F):
#                    debug.toggle()
                    DmxState = 1
                    dmxByte =2
                    
        while(DmxState == 1):    

            # if fixture addr found get RGB from FIFO data
            # get 3 bytes for RGB
            if(dmxByte==lAddrVal+2):
                debug.toggle()
                #get R and G from byte
                R = (d&0x0000FF00)>>8
                G = (d&0xFF000000)>>24
                #get next word from FIFO
                while (not(dmxDataSm.rx_fifo())):pass
                d = dmxDataSm.get()
                B = (d&0x0000FF00)>>8
                rgbLedDuty255(R,G,B)
                DmxState = 2
                
            elif(dmxByte==(lAddrVal+1)):
#                debug.toggle()
                #only get R from byte
                R = (d&0xFF000000)>>24
                #get next word from FIFO
                while (not(dmxDataSm.rx_fifo())):pass
                d = dmxDataSm.get()
                G = (d&0x0000FF00)>>8
                B = (d&0xFF000000)>>24           
                rgbLedDuty255(R,G,B)
                DmxState = 2
               
            else:
                while (dmxDataSm.rx_fifo()==0):pass
                d=dmxDataSm.get()
                dmxByte +=2
                
        # end while(DmxState == 1)    
