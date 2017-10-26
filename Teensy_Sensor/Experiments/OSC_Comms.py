"""
Stackover flow  answer on how to send/recieve data with OSC.
https://stackoverflow.com/questions/32740577/how-do-you-send-a-message-using-pyosc
"""

import OSC
import time, threading

receive_address = '127.0.0.5', 9000


def setup_osc_comms():
    ##VARIABLES
    # tupple with ip, port. i dont use the () but maybe you want ->
    # send_address = ('127.0.0.1', 9000)
    # receive_address = '192.168.0.150', 9000
    #    receive_address = '127.0.0.4', 9000

    # OSC Server. there are three different types of server.
    s = OSC.OSCServer(receive_address) # basic
    ##s = OSC.ThreadingOSCServer(receive_address) # threading
    ##s = OSC.ForkingOSCServer(receive_address) # forking

    # this registers a 'default' handler (for
    # unmatched messages),
    # an /'error' handler, an '/info'
    # handler.
    # And, if the client supports it, a
    # '/subscribe' & '/unsubscribe'
    # handler
    s.addDefaultHandlers()

    # define a message-handler
    # function for the server
    # to call.
    def printing_handler(addr, tags, stuff, source):
        print "---"
        print "received new osc msg from %s" % OSC.getUrlStr(source)
        print "with addr : %s" % addr
        print "typetags %s" % tags
        print "data %s" % stuff
        print "---"

    s.addMsgHandler("/print", printing_handler) # adding our function

    # just
    # checking
    # which
    # handlers
    # we
    # have
    # added
    print "Registered Callback-functions are :"
    for addr in s.getOSCAddressSpace():
        print addr

    # Start
    # OSCServer
    print "\nStarting OSCServer. Use ctrl-C to quit."
    st = threading.Thread( target = s.serve_forever )
    st.start()

    return s, st

s, st = setup_osc_comms()

c = OSC.OSCClient()
c.connect(receive_address)

oscmsg = OSC.OSCMessage()
oscmsg.setAddress("/print")
oscmsg.append('test')
oscmsg.append('test2')
oscmsg.append('test3')

c.send(oscmsg)
