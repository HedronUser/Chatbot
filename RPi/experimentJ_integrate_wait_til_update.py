import serial
from serial import SerialException

import json
import time, threading
import datetime
import decision_engine
import OSC
import os
import sys
import socket, errno


##VARIABLES

drive = 0
strafe = 0
turn = 0

filteredDrive = 0
filteredStrafe = 0
filteredTurn = 0

startMarker = 60
endMarker = 62

receive_address = '192.168.1.87', 9000

sensorPort = "/dev/sensor" #plugged directly into USB port on Pi
controllerPort = "/dev/controller" #plugged directly into USB port on Pi
baudRate = 115200

ser_controller = None
ser_sensor = None

connect_sensor = 0
connect_controller = 0

ip_not_assigned = True
#=====================================

#  Function Definitions

#=====================================

def sendToArduino(sendStr):
  global ser
  ser_controller.write(sendStr)


#======================================

def recvFromArduino():
  global startMarker, endMarker, ser_controller
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser_controller.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    if ord(x) != startMarker:
      ck = ck + x 
      byteCount += 1
    x = ser_controller.read()
  
  return(ck)


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global startMarker, endMarker, connect_controller, ser
    
    msg = ""
    while msg.find("Arduino is ready") == -1:
      try:

          while ser_controller.inWaiting() == 0:
            print "ser_controller.inwaiting passed"
          
  
          msg = recvFromArduino()

          print msg
          print
          connect_controller = 1
          
      except IOError:
          msg = ""
          connect_controller = 0 
          print "ioerror"
          return
        
#======================================

def runTest(td):
  numLoops = len(td)
  waitingForReply = False

  n = 0
  while n < numLoops:

    teststr = td[n]

    if waitingForReply == False:
      sendToArduino(teststr)
      #print "Sent from PC -- LOOP NUM " + str(n) + " TEST STR " + teststr
##      waitingForReply = True
##
##    if waitingForReply == True:
##
##      while ser.inWaiting() == 0:
##        pass
##        
      dataRecvd = recvFromArduino()
      
      n += 1
##      waitingForReply = False

    time.sleep(.01)



def serialconnect_sensor():
  global baudrate, ser_sensor, sensorPort, connect_sensor
  
  try :
      ser_sensor = serial.Serial(sensorPort, baudRate)
      connect_sensor = 1

  except SerialException:
      print "Could not connect to teensy_sensor"
      connect_sensor = 0


def serialconnect_controller():
  global baudrate, ser_controller, controllerPort, connect_controller
  
  try :
      ser_controller = serial.Serial(controllerPort, baudRate)
      connect_controller = 1

  except SerialException:
      print "Could not connect to teensy_controller"
      connect_controller = 0


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
#does nothing but prevents error

def sample_handler(addr, tag, stuff, source):
    h = 1


#======================================

# THE PROGRAM STARTS HERE

#======================================

# OSC Server.

while ip_not_assigned:
  
    try:
        s = OSC.OSCServer(receive_address) # basic

        s.addDefaultHandlers()

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

        time.sleep(1)
        
        ip_not_assigned = False
        
    except socket.error, NameError:
        print "Socket error - cannot assign server programmed ip address"
        


prevTime = 0
interval = 20

while 1 :

    try:
        if connect_sensor == 1:
	  millis = lambda: int(round(time.time()*1000))
 	  if millis() - prevTime > interval:
         ######SERIAL RECEIVE SENSOR -> JSON
            data = ser_sensor.readline().strip().decode('utf8')#reads, strips carriage returns, and decodes to utf8 


            j_sensor = json.loads(data)
  

          
            print j_sensor
	  #print datetime.datetime.now()

          #filter thru decision engine

          #make dictionary with current osc values
            j_osc = {"drive":drive, "strafe": strafe, "turn": turn}

                    
          #filter
            filteredmovement = decision_engine.sensor_filter(j_sensor, j_osc)

          #bind to new variables, this could be skipped
            filteredDrive = filteredmovement["drive"] 

            filteredStrafe = filteredmovement["strafe"] 

            filteredTurn = filteredmovement["turn"] 
	    prevTime = millis()

        else:
            serialconnect_sensor()

    except KeyboardInterrupt:
        break
    except SerialException:
        connect_sensor = 0
        print "serial exception teensy sensor"
        time.sleep(1)
    except ValueError, NameError:
        print "\nJson Parse Error...Retrying"


    try:
        ######SERIAL WRITE TEENSY CONTROLLER
      if connect_controller == 0:
        while(serialconnect_controller() == 0):
          print "teensy Controller connecting"   
          pass

        #waitForArduino()
        #print "teensy Controller active"

      #if sensor connected we want to use filtered data  
      if connect_sensor == 1 and connect_controller == 1:
        testData = []
        testData.append("<drive,127," + str(filteredDrive) + ">")
        testData.append("<turn,127," + str(filteredStrafe) + ">")
        testData.append("<strafe,127," + str(filteredTurn) + ">")

        driver = runTest(testData)

      #if sensors not connected we want to use unfiltered data
      elif connect_sensor == 0 and connect_controller == 1:
        testData = []
        testData.append("<drive,127," + str(drive) + ">")
        testData.append("<turn,127," + str(strafe) + ">")
        testData.append("<strafe,127," + str(turn) + ">")

        print testData
        driver = runTest(testData)
        
    except SerialException:
        connect_controller = 0
        print "serial exception teensy Controller"
        time.sleep(1)
    
    except KeyboardInterrupt:
        break

#if while loop is broken user has exited program so cleanup.
print "\nClosing OSCServer."

s.close()
print "Waiting for Server-thread to finish"

st.join() ##!!!
print "\nClosing SerialPorts."

ser_controller.close
ser_sensor.close
print "Done"

sys.exit

