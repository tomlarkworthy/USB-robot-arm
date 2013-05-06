import usb.core
import usb.util
import time

dev = usb.core.find(idVendor=0x1267, idProduct=0)

def move(j1='', j2='', j3='', j4='', hand='', light=''):

    ba0 = 0
    ba1 = 0
    ba2 = 0

    if(j2 == 'cl'): ba0 += 64
    elif(j2 == 'cc'): ba0 += 128

    if(j3 == 'cl'): ba0 += 16
    elif(j3 == 'cc'): ba0 += 32

    if(j4 == 'cl'): ba0 += 4
    elif(j4 == 'cc'): ba0 += 8

    if(hand == 'open'): ba0 += 2
    elif(hand == 'close'): ba0 += 1

    if(j1 == 'cl'): ba1 = 1
    elif(j1 == 'cc'): ba1 = 2

    if(light == 'on'): ba2 = 1

    dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))

#move('cl')
move('cc')
time.sleep(5)
#move('cc')
#time.sleep(4)


#move('cl','cl','cl','cl','open','off')
#time.sleep(1)
#move('cc','cc','cc','cc','close','on')
#time.sleep(1)

move()


    
