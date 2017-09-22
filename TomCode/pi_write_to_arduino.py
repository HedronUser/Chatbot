#pi user name: pi
#pi password: hydronics

#find arduino serial using command: ls /dev/tty*
#might be dev/ttyACM0 or ttyACM1 or ttyUSB0

#Disable Auto Reset on Serial connection:
#Stick a 120 ohm resistor in the headers between 5v and reset on the arduino

#autostart the script
# sudo nano /etc/rc.local
# and then replace the IP dummy script with
# sudo python /home/pi/lots/pi_write_to_arduino.py


# pin 3 - first panel warm
# pin 5 - 2nd panel cool
# pin 6 - first panel cool
# pin 9 - 2nd panel warm
# pin 10 - baclight (one pin controls both mosfets)

#to run after putty quits pi@raspberrypi:~$ nohup python pi_write_to_arduino.py &

#!/usr/bin/env python
import serial
from time import sleep

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

print "Serial is open: " + str(ser.isOpen())

# print "Now Writing"
# ser.write("6:12,5:50") #pin 6, PWM 100
# ser.write('\n')
#
# print "Did write, now read"
# x = ser.readline()
# print "got '" + x + "'" #got '6:100'
#
# ser.close()

ser.write("3:"+ str(0) + "\n")
ser.write("5:"+ str(0) + "\n")
ser.write("6:"+ str(0) + "\n")
ser.write("9:"+ str(0) + "\n")
ser.write("10:"+ str(0) + "\n")

while True:
    # ser.write("3:"+ str(50) + "\n")
    # sleep(1)
    # ser.write("3:"+ str(0) + "\n")
    # sleep(1)
    # ser.write("5:"+ str(50) + "\n")
    # sleep(1)
    # ser.write("5:"+ str(0) + "\n")
    # sleep(1)
    # ser.write("6:"+ str(50) + "\n")
    # sleep(1)
    # ser.write("6:"+ str(0) + "\n")
    # sleep(1)
    # ser.write("9:"+ str(50) + "\n")
    # sleep(1)
    # ser.write("9:"+ str(0) + "\n")
    # sleep(1)
    # ser.write("10:"+ str(50) + "\n")
    # sleep(1)
    # ser.write("10:"+ str(0) + "\n")
    # sleep(1)
    for pwm in range(0, 100, 1):
        ser.write("3:"+ str(pwm) + "\n")
        sleep(0.1)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("3:"+ str(pwm) + "\n")
        sleep(0.1)

    for pwm in range(0, 100, 1):
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)

    for pwm in range(0, 100, 1):
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.15)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.15)

    for pwm in range(0, 100, 1):
        ser.write("5:"+ str(pwm) + "\n")
        sleep(0.15)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("5:"+ str(pwm) + "\n")
        sleep(0.15)


#light both warm and warm together
    for pwm in range(0, 100, 1):
        ser.write("3:"+ str(pwm) + "\n")
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("3:"+ str(pwm) + "\n")
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)


    for pwm in range(0, 100, 1):
        ser.write("5:"+ str(pwm) + "\n")
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.15)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("5:"+ str(pwm) + "\n")
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.15)


#light both warm and cool together
    for pwm in range(0, 100, 1):
        ser.write("3:"+ str(pwm) + "\n")
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.1)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("3:"+ str(pwm) + "\n")
        ser.write("6:"+ str(pwm) + "\n")
        sleep(0.1)


    for pwm in range(0, 100, 1):
        ser.write("5:"+ str(pwm) + "\n")
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)
    sleep(2)
    for pwm in range(0, 254):  #back light ramp up
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    sleep(.5)
    for pwm in range(254,-1,-1): #back light ramp down
        ser.write("10:"+ str(pwm) + "\n")
        sleep(0.01)
    for pwm in range(100, -1, -1):
        ser.write("5:"+ str(pwm) + "\n")
        ser.write("9:"+ str(pwm) + "\n")
        sleep(0.1)
