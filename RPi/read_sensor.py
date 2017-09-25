#!/usr/bin/env python

import serial

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )



print "Serial is open: " + str(ser.isOpen())
front = 'front'
right = 'right'
rear = 'rear'
left = 'left'


while True:
    
    x = ser.readline()
    words = x.split()
    
##    print words[0]
##    print words[2]
    
    if (words[0] == 'front' and words[2] == '1'):
        print 'Obstacle in front'
    elif (words[0] == 'front' and words[2] == '0'):
        print 'Obstacle free'
        



