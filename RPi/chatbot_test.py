
""" receiving OSC with pyOSC
https://trac.v2.nl/wiki/pyOSC
example by www.ixi-audio.net based on pyOSC documentation

this is a very basic example, for detailed info on pyOSC functionality check the OSC.py file
or run pydoc pyOSC.py. you can also get the docs by opening a python shell and doing
>>> import OSC
>>> help(OSC)
"""

import serial
from serial import SerialException
import json

import OSC
import time, threading

import os

import decision_engine

##################
#GLOBAL VARIABLES#
#   AND SETUP    #
##################



#####COMMS VARIABLES
##
#### SET DEFAULT ARDUINO DEVICE PATHS
#port_sensor = "/dev/ttyACM0"
#port_controller = "/dev/ttyAMA0"
port_controller = "/dev/serial0"
#port_controller = "/dev/ttyACM1" #not sure which one


##### FOR SETTING A DIFFERENT DEVICE BATH IN ENVIRONMENT VARIABLES
### In a bash shell use:
### > export ARDUINO_PATH=$path_to_device
### In windows power shell useL
### > set ARDUINO_$_PATH='$path_to_device'
if "ARDUINO_SENSOR_PATH" in os.environ:
    port = os.environ["ARDUINO_SENSOR_PATH"]
if "ARDUINO_CONTROLLER_PATH" in os.environ:
    port = os.environ["ARDUINO_CONTROLLER_PATH"]

### SET ARDUINO SERIAL BAUDRATE
baudrate = 57600
##
#####END COMMS VARIABLES#####


#####SERIAL COMMS SETUP######
###
def setup_serial(port, baudrate):
    ##VARIABLES
    #port = port
    #baudrate = baudrate
    parity=serial.PARITY_NONE
    stopbits=serial.STOPBITS_ONE
    bytesize=serial.EIGHTBITS
    timeout=1


    ##RETURN SERIAL
    ser = serial.Serial(
        port = port,
        baudrate = baudrate,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        timeout=1
        )
    return ser

#ser_sensor = setup_serial(port_sensor, baudrate)
try :
    ser_controller = setup_serial(port_controller, baudrate)

except SerialException:
    print "Could not connect to device"
    port_controller = "/dev/ttyS0" #not sure which one
    ser_controller = setup_serial(port_controller, baudrate)

#give time for arduino to setup comms
time.sleep(2)

###
#############################

startMarker = 254
endMarker = 255
specialByte = 253



##########BEGIN MOVEMENT COUNTING########
##
##count_triplet = [0,0,0]
##movement_triplet = [0,0,0]
##last_avg_timestamp = time.time()
##time_delta = .2 ## 200 ms
##
##
##def count_movements(count_triplet, movement_triplet, **kwargs):
##    """
##    The movement_triplet takes the form (drive, strafe, turn)
##    The count_triplet, takes a similar form (drive_counts, strafe_counts, turn_counts)
##
##    for every drive, strafe, or turn, event handled, increments the count
##    and addeds to the movement_triplet total movement count
##    """
##    for key, value in kwargs.items():
##        if key == "drive":
##            movement_triplet[0], count_triplet[0] = \
##                movement_triplet[0]+value, count_triplet[0]+1
##        if key == "strafe":
##            movement_triplet[1], count_triplet[1] = \
##                movement_triplet[1]+value, count_triplet[1]+1
##        if key == "turn":
##            movement_triplet[2], count_triplet[2] = \
##                movement_triplet[2]+value, count_triplet[2]+1
##    return count_triplet, movement_triplet
##
##def avg_movement(count_triplet,movement_triplet):
##    """
##    The movement_triplet takes the form (drive, strafe, turn)
##    The count_triplet, takes a similar form (drive_counts, strafe_counts, turn_counts)
##
##    Divides each total movement sum by the total counts of recorded movement messages
##    """
##    #make sure no div by zero
##    if count_triplet[0] <= 0: count_triplet[0]=1
##    if count_triplet[1] <= 0: count_triplet[1]=1
##    if count_triplet[2] <= 0: count_triplet[2]=1
##    # take simple average
##    movement_triplet[0] = movement_triplet[0] / count_triplet[0]
##    movement_triplet[1] = movement_triplet[1] / count_triplet[1]
##    movement_triplet[2] = movement_triplet[2] / count_triplet[2]
##    # checks to make sure values arent greater than abs(value)>=128?
##    # TODO: CHECKS
##
##    return movement_triplet

#count_triplet = [0,0,0]
#movement_triplet = [0,0,0]
#last_avg_timestamp = time.time()
#time_delta = .2 ##200 ms

### needed for movement counting & averaging

##
####END MOVEMENT COUNTING#########################

####OSC COMMS SETUP##########
###


##VARIABLES
# tupple with ip, port. i dont use the () but maybe you want -> send_address = ('127.0.0.1', 9000)
# receive_address = '192.168.0.150', 9000
receive_address = '192.168.1.52', 9000

# OSC Server. there are three different types of server.
s = OSC.OSCServer(receive_address) # basic
##s = OSC.ThreadingOSCServer(receive_address) # threading
##s = OSC.ForkingOSCServer(receive_address) # forking

# this registers a 'default' handler (for unmatched messages),
# an /'error' handler, an '/info' handler.
# And, if the client supports it, a '/subscribe' & '/unsubscribe' handler
s.addDefaultHandlers()

# define a message-handler function for the server to call.
##def printing_handler(addr, tags, stuff, source):
##    print "---"
##    print "received new osc msg from %s" % OSC.getUrlStr(source)
##    print "with addr : %s" % addr
##    print "typetags %s" % tags
##    print "data %s" % stuff
##    print "---"

drive = 0
strafe = 0
turn = 0
def drive_handler(addr, tags, stuff, source):
    global drive
    drive = stuff[0]
    
##    print "drive"
##    print "addr: %s" % addr
##    print "stuff: %s" % stuff

##
def strafe_handler(addr, tags, stuff, source):
    global strafe
    strafe = stuff[0]


##    print "strafe"
##    print "addr: %s" % addr
##    print "stuff: %s" % stuff
    
def turn_handler(addr, tags, stuff, source):
    global turn
    turn = stuff[0]
    
##    print "turn"
##    print "addr: %s" % addr
##    print "stuff: %s" % stuff

##def drive_handler(addr, tags, stuff, source):
##    global count_triplet
##    global movement_triplet
##    count_triplet, movement_triplet = \
##            count_movements(count_triplet, movement_triplet,
##                            drive = int(stuff[0]))
##    
####    print "drive"
####    print "addr: %s" % addr
####    print "stuff: %s" % stuff
##
####
##def strafe_handler(addr, tags, stuff, source):
##    global count_triplet
##    global movement_triplet
##    count_triplet, movement_triplet = \
##            count_movements(count_triplet, movement_triplet,
##                            strafe = int(stuff[0]))
##
####    print "strafe"
####    print "addr: %s" % addr
####    print "stuff: %s" % stuff
##    
##def turn_handler(addr, tags, stuff, source):
##    global count_triplet
##    global movement_triplet
##    count_triplet, movement_triplet = \
##            count_movements(count_triplet, movement_triplet,
##                            turn = int(stuff[0]))
##    print "turn"
##    print "addr: %s" % addr
##    print "stuff: %s" % stuff

#does nothing but prevents error
def sample_handler(addr, tag, stuff, source):
    h = 1

##s.addMsgHandler("/print", printing_handler) # adding our function
s.addMsgHandler("/drive", drive_handler)
s.addMsgHandler("/strafe", strafe_handler)
s.addMsgHandler("/turn", turn_handler)
s.addMsgHandler("/_samplerate", sample_handler)


# just checking which handlers we have added
print "Registered Callback-functions are :"
for addr in s.getOSCAddressSpace():
    print addr

# Start OSCServer
print "\nStarting OSCServer. Use ctrl-C to quit."
st = threading.Thread( target = s.serve_forever )
st.start()



time.sleep(.1)
###
########################
#=====================================

#  Serial send Function Definitions

#=====================================

def sendToArduino(sendStr):
  global startMarker, endMarker
  txLen = chr(len(sendStr))
  adjSendStr = encodeHighBytes(sendStr)
  adjSendStr = chr(startMarker) + txLen + adjSendStr + chr(endMarker)
  ser_controller.write(adjSendStr)


#======================================

def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser_controller.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    ck = ck + x 
    x = ser_controller.read()
    byteCount += 1
    
  # save the end marker byte
  ck = ck + x 
  
  returnData = []
  returnData.append(ord(ck[1]))
  returnData.append(decodeHighBytes(ck))
#  print "RETURNDATA " + str(returnData[0])
  
  return(returnData)

#======================================

def encodeHighBytes(inStr):
  global specialByte
  
  outStr = ""
  s = len(inStr)
  
  for n in range(0, s):
    x = ord(inStr[n])
    
    if x >= specialByte:
       outStr = outStr + chr(specialByte)
       outStr = outStr + chr(x - specialByte)
    else:
       outStr = outStr + chr(x)
       
#  print "encINSTR  " + bytesToString(inStr)
#  print "encOUTSTR " + bytesToString(outStr)

  return(outStr)


#======================================

def decodeHighBytes(inStr):

  global specialByte
  
  outStr = ""
  n = 0
  
  while n < len(inStr):
     if ord(inStr[n]) == specialByte:
        n += 1
        x = chr(specialByte + ord(inStr[n]))
     else:
        x = inStr[n]
     outStr = outStr + x
     n += 1
     
  print "decINSTR  " + bytesToString(inStr)
  print "decOUTSTR " + bytesToString(outStr)

  return(outStr)


#======================================

def displayData(data):

  n = len(data) - 3

  print "NUM BYTES SENT->   " + str(ord(data[1]))
  print "DATA RECVD BYTES-> " + bytesToString(data[2:-1])
  print "DATA RECVD CHARS-> " + data[2: -1]


#======================================

def bytesToString(data):

  byteString = ""
  n = len(data)
  
  for s in range(0, n):
    byteString = byteString + str(ord(data[s]))
    byteString = byteString + "-"
    
  return(byteString)


#======================================

def displayDebug(debugStr):

   n = len(debugStr) - 3
   print "DEBUG MSG-> " + debugStr[2: -1]


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global endMarker
    
    msg = ""
    while msg.find("Arduino Ready") == -1:

      while ser_controller.inWaiting() == 0:
        x = 'z'

      # then wait until an end marker is received from the Arduino to make sure it is ready to proceed
      x = "z"
      while ord(x) != endMarker: # gets the initial debugMessage
        x = ser_controller.read()
        msg = msg + x


      displayDebug(msg)
      print
      
##waitForArduino()
##
##
##print "Arduino Ready"



#############
#Begin Loops#
#############

##Listen over serial

while True:

######SERIAL RECEIVE SENSOR -> JSON
###
#    time.sleep(.1)
#    data = ser_sensor.readline().strip().decode('utf8')#reads, strips carriage returns, and decodes to utf8 
#    j_sensor = json.loads(data)
  
#############################
    
##    print "\nGETTING HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    #print ser_controller.readline()
######SERIAL WRITE CONTROLLER
###
### Check if the time_delta has elapsed
##    if (time.time() - last_avg_timestamp) > time_delta:
##        movement_triplet = \
##            avg_movement(count_triplet, movement_triplet)
##	print movement_triplet

##      s_movement = {"drive":movement_triplet[0],
##                      "strafe":movement_triplet[1],
##                      "turn":movement_triplet[2]}
##    s_movement = {"d":round(drive,3), "s":round(strafe,3),"t":round(turn,3)}
##    j_movement = json.dumps(s_movement)
##    print s_movement
##    print j_movement
    ##SERIAL SEND CONTROLLER
#    j_movement = decision_engine.sensor_filter(j_sensor,j_osc)
    #ser_controller.write(s_movement)
##    ser_controller.write(j_movement)
    #ser_controller.write("drive")
    #print "sent message"

##        ##Reset Counts
##        count_triplet = [0,0,0]
##        movement_triplet = [0,0,0]
##        
##        ##Reset Time Stamp
##        last_avg_timestamp = time.time()

###
###############################

    testData = []
    testData.append("drive, " + str(round(drive, 3))) #a
    testData.append("strafe, " + str(round(strafe, 3)))
    testData.append("turn, " + str(round(turn, 3)))


    sendToArduino(testData)
    
 
    print testData

##      if ser_controller.inWaiting > 0
##        dataRecvd = recvFromArduino()
##
##        if dataRecvd[0] == 0:
##          displayDebug(dataRecvd[1])
##
##        if dataRecvd[0] > 0:
##          displayData(dataRecvd[1])
##          print "Reply Received"
##          n += 1s

    time.sleep(0.1)




try :
    while 1 :
        time.sleep(5)

except KeyboardInterrupt :
    print "\nClosing OSCServer."
    s.close()
    print "Waiting for Server-thread to finish"
    st.join() ##!!!
    print "Done"


