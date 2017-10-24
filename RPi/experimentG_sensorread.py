import serial
from serial import SerialException

import json
import time
import datetime

sensorPort = "/dev/sensor" #plugged directly into USB port on Pi
controllerPort = "/dev/controller" #plugged directly into USB port on Pi
baudRate = 115200
ser_controller = None
ser_sensor = None

connect = 0


def serialconnect_sensor():
  global baudrate, ser_sensor, sensorPort, connect
  
  try :
      ser_sensor = serial.Serial(sensorPort, baudRate)
      connect = 1

  except SerialException:
      print "Could not connect to teensy_sensor"
      connect = 0



while 1 :

    try:
        if connect == 1:

          ######SERIAL RECEIVE SENSOR -> JSON
          data = ser_sensor.readline().strip().decode('utf8')#reads, strips carriage returns, and decodes to utf8 
          j_sensor = json.loads(data)
          print j_sensor
          print datetime.datetime.now()
          
        else:
            serialconnect_sensor()

    except KeyboardInterrupt:
        print "\nClosing OSCServer."
        s.close()
        print "\nClosing SerialPort."
        ser_controller.close
        ser_sensor.close
        print "Waiting for Server-thread to finish"
        st.join() ##!!!
        print "Done"
    except SerialException:
        connect = 0
        time.sleep(5)

        
