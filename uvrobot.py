import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
import cv2                                 #Import cv2 library
from yolo_inference import main            #Import inference function
from ultralytics.yolo.engine.model import YOLO #import YOLO
from helpers import *                          #import helpers

door_model_path = "models/best.pt"             #path to model
model = YOLO(door_model_path)                  # create model

# Disable warnings
GPIO.setwarnings(False)

# programming the GPIO by BCM pin numbers
GPIO.setmode(GPIO.BCM)

# Ultrasonic sensor pins
TRIG=11
ECHO=18

# LED array pin
led=16

# Buzzer pin
buzzer=25

# PIR sensor pin
PIR=21

# Motor driver input pins
m11=17
m12=27
m21=22
m22=23
ENA=13
ENB=12

# Setup GPIO pins
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(led,GPIO.OUT)                  
GPIO.setup(m11,GPIO.OUT)
GPIO.setup(m12,GPIO.OUT)
GPIO.setup(m21,GPIO.OUT)
GPIO.setup(m22,GPIO.OUT)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(ENB,GPIO.OUT)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setup(PIR, GPIO.IN)

en1=GPIO.PWM(ENA, 1000)
en2=GPIO.PWM(ENB, 1000)

en1.start(60)
en2.start(60)

def stop():
    print("stop")
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

def forward():
    en1.ChangeDutyCycle(60)
    en2.ChangeDutyCycle(60)
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print("Forward")

def back():
    en1.ChangeDutyCycle(50)
    en2.ChangeDutyCycle(50)
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 1)
    print("back")

def left():
    en1.ChangeDutyCycle(50)
    en2.ChangeDutyCycle(50)
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print("left")

def right():
    en1.ChangeDutyCycle(50)
    en2.ChangeDutyCycle(50)
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    print("right")


def get_distance ():
    global TRIG, ECHO                                                   # Allow access to 'trig' and 'echo' constants

    if GPIO.input (ECHO):                                               # If the 'Echo' pin is already high
        return (100)                                                    # then exit with 100 (sensor fault)

    distance = 0                                                        # Set initial distance to zero

    GPIO.output (TRIG,False)                                            # Ensure the 'Trig' pin is low for at
    time.sleep (0.05)                                                   # least 50mS (recommended re-sample time)

    GPIO.output (TRIG,True)                                             # Turn on the 'Trig' pin for 10uS (ish!)
    dummy_variable = 0                                                  # No need to use the 'time' module here,
    dummy_variable = 0                                                  # a couple of 'dummy' statements will do fine
    
    GPIO.output (TRIG,False)                                            # Turn off the 'Trig' pin
    time1, time2 = time.time(), time.time()                             # Set inital time values to current time
    
    while not GPIO.input (ECHO):                                        # Wait for the start of the 'Echo' pulse
        time1 = time.time()                                             # Get the time the 'Echo' pin goes high
        if time1 - time2 > 0.02:                                        # If the 'Echo' pin doesn't go high after 20mS
            distance = 100                                              # then set 'distance' to 100
            break                                                       # and break out of the loop
        
    if distance == 100:                                                 # If a sensor error has occurred
        return (distance)                                               # then exit with 100 (sensor fault)
    
    while GPIO.input (ECHO):                                            # Otherwise, wait for the 'Echo' pin to go low
        time2 = time.time()                                             # Get the time the 'Echo' pin goes low
        if time2 - time1 > 0.02:                                        # If the 'Echo' pin doesn't go low after 20mS
            distance = 100                                              # then ignore it and set 'distance' to 100
            break                                                       # and break out of the loop
        
    if distance == 100:                                                 # If a sensor error has occurred
        return (distance)                                               # then exit with 100 (sensor fault)
        
                                                                        # Sound travels at approximately 2.95uS per mm
                                                                        # and the reflected sound has travelled twice
                                                                        # the distance we need to measure (sound out,
                                                                        # bounced off object, sound returned)
                                                                        
    distance = (time2 - time1) / 0.00000295 / 2 / 10                    # Convert the timer values into centimetres
    return (distance)


stop()
count=0

# Function to move robot
def move():
     global count
     i=0
     avgDistance=get_distance()
     print(avgDistance)

     flag=0

     if avgDistance <= 18:      #Check whether the distance is within 18 cm range
        count=count+1
        stop()
        time.sleep(1)
        back()
        time.sleep(1.8)

        if (count%3 ==1) & (flag==0):
         right()
         flag=1
        else:
         left()
         flag=0
        time.sleep(1.5)
        stop()
        time.sleep(1)
     else:
        forward()
        flag=0
        

## Initializing video stream
print('[INFO] starting video stream...')
cap = cv2.VideoCapture(0)
time.sleep(1.0)

try:

    while True:
        i=GPIO.input(PIR)
        ret, frame = cap.read()
        
        # Sound the buzzer for 1 second every 5 seconds
        if i==1:
            print("human detected")
            stop()
            GPIO.output(led, 0)
            back()
            time.sleep(0.5)
            GPIO.output(buzzer, 1)
            time.sleep(1)
            GPIO.output(buzzer, 0)
            time.sleep(4)
        else:
            GPIO.output(led, 1)
            back
            time.sleep(1)
            move()
            time.sleep(2)
            stop()
            time.sleep(10)

            pred = main(model, frame)
            print(pred)

            if pred is not None:
                stop()
                time.sleep(1)
                (startX, startY, endX, endY) = round(pred['x1']), round(pred['y1']), round(pred['x2']), round(pred['y2'])
                color = (0, 255, 0)
                label = f"{pred['class']} {float(pred['conf']):0.2f}"

                cv2.putText(frame, label, (startX, startY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
                cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
 
                if endX >= 600:
                    stop()
                    time.sleep(0.1)
                    print("turn right")
                    time.sleep(0.2)
                elif startX <= 50:
                    stop()
                    time.sleep(0.1)
                    print("turn left")
                    left()
                    time.sleep(0.2)
                else:
                    print("Move forward")
                    forward()
                    time.sleep(0.5)
                    dist_obj =  get_distance()
                    if dist_obj <= 20:
                        stop()
                        time.sleep(2 * 60)

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            
except KeyboardInterrupt:
    print("Stopped by User")
finally:
    GPIO.cleanup()
    cv2.destroyAllWindows()
