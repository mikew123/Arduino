import machine
import utime
import time
import ustruct
import sys

###############################################################################
# Constants

# Registers
REG_DEVID = 0x00
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Other constants
DEVID = 0xE5

###############################################################################
# Settings

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
    
    
##############################################################
    
clearDisplayRam()

prevButtons = 0b0000

# these variables need to be non-volatile and restored at reset
dstate = "off" # display state
lstate = "off" # led state

addrVal = 1
redVal = 0
grnVal = 0
bluVal = 0

prevButtons = 0

while True:
    buttons = readButtonsX4()
    if(buttons != prevButtons):
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
                if(lstate == "off"):
                    print("LED and display OFF")
                elif(lstate == "addr"):
                    print("LED ON, DMX mode")
                elif((lstate == "red")or(lstate == "grn")or(lstate == "blu")):
                    print("LED ON, RGB mode")

        prevButtons = buttons
        # end run code once when button pushed
