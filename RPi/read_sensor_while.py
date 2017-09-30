
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




# tupple with ip, port. i dont use the () but maybe you want -> send_address = ('127.0.0.1', 9000)
receive_address = '192.168.0.150', 9000


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

##Listen over serial

while True:
    
    x = ser.readline()
    words = x.split()
    

    
##    print words[0]
##    print words[2]
##    print words[3]
##    print words[5]
    
##    
    if (words[0] == 'front' and words[2] == '1'):
        print 'Obstacle in front'
    elif (words[0] == 'front' and words[2] == '0'):
        print 'Obstacle free'
    if (words[0] == 'right' and words[2] == '1'):
        print 'Obstacle on right'
    elif (words[0] == 'right' and words[2] == '0'):
        print 'Obstacle free'
    if (words[0] == 'rear' and words[2] == '1'):
        print 'Obstacle in rear'
    elif (words[0] == 'rear' and words[2] == '0'):
        print 'Obstacle free'
    if (words[0] == 'left' and words[2] == '1'):
        print 'Obstacle on left'
    elif (words[0] == 'left' and words[2] == '0'):
        print 'Obstacle free'        


try :
    while 1 :
        time.sleep(5)

except KeyboardInterrupt :
    print "\nClosing OSCServer."
    s.close()
    print "Waiting for Server-thread to finish"
    st.join() ##!!!
    print "Done"
        

