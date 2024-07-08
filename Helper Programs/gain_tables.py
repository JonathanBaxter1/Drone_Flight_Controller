import math

def int2byte(number):
    return ".db $"+format(int(number) & 0xFF,'#04x')[2:].upper()

for i in range(0, 256):
    if (i < 128):
        number = i
    else:
        number = i-256
    
    Pnumber = number
    if number > 90:
        Pnumber = 90
    elif number < -90:
        Pnumber = -90
    Pgain = round(Pnumber*0.17454148769)
    Igain = round(number*0.0893652414)
    Dgain = round(number*0.288134375)
    if Dgain > 32:
        Dgain = 32
    if Dgain < -32:
        Dgain = -32
    print(int2byte(Pgain))
    #print(int2byte(Igain))
    #print(int2byte(Dgain))