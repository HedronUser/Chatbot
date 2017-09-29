
""" receiving OSC with pyOSC
https://trac.v2.nl/wiki/pyOSC
example by www.ixi-audio.net based on pyOSC documentation

this is a very basic example, for detailed info on pyOSC functionality check the OSC.py file
or run pydoc pyOSC.py. you can also get the docs by opening a python shell and doing
>>> import OSC
>>> help(OSC)
"""

import serial
import OSC
import time, threading
import os
import json

##################
#GLOBAL VARIABLES#
#   AND SETUP    #
##################


#####SERIAL COMMS SETUP######
###
def setup_serial():
    ##VARIABLES
    port = "/dev/ttyACM0"
    baudrate = 9600
    parity=serial.PARITY_NONE
    stopbits=serial.STOPBITS_ONE
    bytesize=serial.EIGHTBITS
    timeout=1

    ##### FOR SETTING A DIFFERENT ARDUINO DEVICE PATH
    ### In a bash shell use:
    ### > export ARDUINO_PATH=$path_to_device
    ### In windows power shell useL
    ### > set ARDUINO_PATH='$path_to_device'
    if "ARDUINO_PATH" in os.environ:
        port = os.environ["ARDUINO_PATH"]

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
ser = setup_serial()
###
#############################

  # TODO not sure if necessary:
  #  print "Serial is open: " + str(ser.isOpen())
  #  front = 'front'
  #  right = 'right'
  #  rear = 'rear'
  #  left = 'left'

####OSC COMMS SETUP##########
###
def setup_osc_comms():
    ##VARIABLES
    # tupple with ip, port. i dont use the () but maybe you want -> send_address = ('127.0.0.1', 9000)
    # receive_address = '192.168.0.150', 9000
    receive_address = '127.0.0.2', 9000

    # OSC Server. there are three different types of server.
    s = OSC.OSCServer(receive_address) # basic
    ##s = OSC.ThreadingOSCServer(receive_address) # threading
    ##s = OSC.ForkingOSCServer(receive_address) # forking

    # this registers a 'default' handler (for unmatched messages),
    # an /'error' handler, an '/info' handler.
    # And, if the client supports it, a '/subscribe' & '/unsubscribe' handler
    s.addDefaultHandlers()

    # define a message-handler function for the server to call.
    def printing_handler(addr, tags, stuff, source):
        print "---"
        print "received new osc msg from %s" % OSC.getUrlStr(source)
        print "with addr : %s" % addr
        print "typetags %s" % tags
        print "data %s" % stuff
        print "---"

    s.addMsgHandler("/print", printing_handler) # adding our function

    # just checking which handlers we have added
    print "Registered Callback-functions are :"
    for addr in s.getOSCAddressSpace():
        print addr

    # Start OSCServer
    print "\nStarting OSCServer. Use ctrl-C to quit."
    st = threading.Thread( target = s.serve_forever )
    st.start()
###
########################

#############
#Begin Loops#
#############



##Listen over serial
time.sleep(.1)

while True:

######SERIAL RECEIVE -> JSON
###
    time.sleep(.005)
    data = ser.readline().strip().decode('utf8')#reads, strips carriage returns, and decodes to utf8 
    j = json.loads(data)
    print(j["channel_data"][0])
###
#############################



##    if (words[0] == 'front' and words[2] == '1'):
##        print 'Obstacle in front'
##    elif (words[0] == 'front' and words[2] == '0'):
##        print 'Obstacle free'
##    if (words[0] == 'right' and words[2] == '1'):
##        print 'Obstacle on right'
##    elif (words[0] == 'right' and words[2] == '0'):
##        print 'Obstacle free'
##    if (words[0] == 'rear' and words[2] == '1'):
##        print 'Obstacle in rear'
##    elif (words[0] == 'rear' and words[2] == '0'):
##        print 'Obstacle free'
##    if (words[0] == 'left' and words[2] == '1'):
##        print 'Obstacle on left'
##    elif (words[0] == 'left' and words[2] == '0'):
##        print 'Obstacle free'


try :
    while 1 :
        time.sleep(5)

except KeyboardInterrupt :
    print "\nClosing OSCServer."
    s.close()
    print "Waiting for Server-thread to finish"
    st.join() ##!!!
    print "Done"


