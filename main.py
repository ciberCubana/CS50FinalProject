
#Sterling Nolasco And Neisy 

# CS50xMiami 


import time
import sys
import select
import termios
import tty
import pynput
import RPi.GPIO as GPIO
from pynput import keyboard

# Import the Robot.py file (must be in the same directory as this file!).
import Robot

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

LEFT_TRIM   = 0
RIGHT_TRIM  = 0

# Create an instance of the robot with the specified trim values.
# Not shown are other optional parameters:
#  - addr: The I2C address of the motor HAT, default is 0x60.
#  - left_id: The ID of the left motor, default is 1.
#  - right_id: The ID of the right motor, default is 2.
robot = Robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    
    return distance

'''def on_press(key):
    try:    
        robot.forward(70)
        format(key.char)
        
    except AttributeError:
        format(key)'''

#def on_release(key):
    #robot.stop()
    #format(key)
    
    #if key == keyboard.Key.esc:
        # Stop listener
        #return False

# Collect events until released
#with keyboard.Listener(
      #  on_press=on_press,
        #on_release=on_release) as listener:
    #listener.join()


def getkey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
  

# Set the trim offset for each motor (left and right).  This is a value that
# will offset the speed of movement of each motor in order to make them both
# move at the same desired speed.  Because there's no feedback the robot doesn't
# know how fast each motor is spinning and the robot can pull to a side if one
# motor spins faster than the other motor.  To determine the trim values move the
# robot forward slowly (around 100 speed) and watch if it veers to the left or
# right.  If it veers left then the _right_ motor is spinning faster so try
# setting RIGHT_TRIM to a small negative value, like -5, to slow down the right
# motor.  Likewise if it veers right then adjust the _left_ motor trim to a small
# negative value.  Increase or decrease the trim value until the bot moves
# straight forward/backward.


# Now move the robot around!
# Each call below takes two parameters:
#  - speed: The speed of the movement, a value from 0-255.  The higher the value
#           the faster the movement.  You need to start with a value around 100
#           to get enough torque to move the robot.
#  - time (seconds):  Amount of time to perform the movement.  After moving for
#                     this amount of seconds the robot will stop.  This parameter
#                     is optional and if not specified the robot will start moving
#                     forever.




while True:
        key = getkey()
        dist = distance()
        #print("Distance meas: %.1f" % dist)
        if key == 's':
            #print("Atras..")
            robot.backward(100)
            GPIO.output(21,GPIO.HIGH) # turn on LED while going Backwards
            time.sleep(0.5)
            GPIO.output(21,GPIO.LOW)  # turn off LED while going Backwards
            time.sleep(0.5)
            GPIO.output(21,GPIO.HIGH) # turn on LED while going Backwards

        elif key == 'w' :
                #print("Alante...")
                robot.forward(110)
        elif key == 'd' :
                #print("Derecha..")
                robot.right(110)
        elif key == 'a':
                #print("Izquierda...")
                robot.left(110)
        elif key == 'q':
            #print 'stop'
            robot.stop()
        elif key == 'x':
             break
        if dist < 25.0:
            robot.stop()

       # with keyboard.Listener(
            #  on_press=on_press,
               # on_release=on_release) as listener:
            #listener.join()
        
'''robot.forward(120,3.0)   # Move forward at speed 120 for second.
print("Going Forward..")

if robot.forward:
    GPIO.output(21,GPIO.HIGH) #Turn on the LED Light and keep it stady while going Forward

robot.backward(120,8.0)   # Move forward at speed 120 for second.
print("Going Backward..")

while robot.backward:
        print("Going Backwards. ciclo..")
        GPIO.output(21,GPIO.LOW) # turn off LED while going Backwards
        time.sleep(0.5)
        GPIO.output(21,GPIO.HIGH)  
        time.sleep(0.5)
        GPIO.output(21,GPIO.LOW) # turn off LED while going Backwards
        time.sleep(0.5)
        GPIO.output(21,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(21,GPIO.LOW) # turn off LED while going Backwards
        




time.sleep(2.0)   # Pause for a few seconds while the robot spins (you could do

robot.stop()      # Stop the robot from moving.'''

