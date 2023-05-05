import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from time import sleep
from math import ceil
from picamera import PiCamera

if _name_ == '_main_' :

    GPIO.setmode(GPIO.BCM)

    trig = 18
    echo = 24
    GPIO.setup(trig,GPIO.OUT)
    GPIO.setup(echo,GPIO.IN)

    enA = 12 #pwm
    enB = 13 #pwm
    in1 = 5
    in2 = 6
    in3 = 27
    in4 = 22
    GPIO.setup(enA,GPIO.OUT)
    GPIO.setup(enB,GPIO.OUT)
    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(in3,GPIO.OUT)
    GPIO.setup(in4,GPIO.OUT)
    pwma = GPIO.PWM(enA,1000)
    pwmb = GPIO.PWM(enB,1000)
    pwma.start(0)
    pwmb.start(0)        



    spensor = 16
    GPIO.setup(spensor, GPIO.IN)
    count = 0
    pre_state = 0
    distance = 0

    cam = PiCamera()
    cam.rotation = 180

def ultra():
    GPIO.output(trig,True)
    sleep(0.000001)
    GPIO.output(trig,False)
    while GPIO.input(echo) == 0 :
        start = time.time()
    while GPIO.input(echo) == 1 :
        stop = time.time()
    t = stop - start
    d = (t*34300)/2
    #print(d)
    if not (d>=5 and d<8):
        print(d,"not aligned")
    else :
        return
    
def forward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    pwma.start(0)
    pwmb.start(0)
    pwma.ChangeDutyCycle(100)
    pwmb.ChangeDutyCycle(100)
    
        
def stop_bot() :
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

        
def odometer():

    global pre_state,count,distance
    if GPIO.input(spensor)==1 and pre_state==0:
        count+=1
        pre_state = 1
    if pre_state==1 and GPIO.input(spensor)==0 :
        count+=1
        pre_state = 0
    distance = count*18.84/40
    #print("distance travelled =",distance)
    return distance
        
def photo():
    cap=cv2.VideoCapture(0)
    lower_range=np.array([157,144,49])
    upper_range=np.array([179,255,255])
    ret,frame=cap.read()
    frame=cv2.resize(frame,(640,480))
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,lower_range,upper_range)
    _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
    cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for c in cnts:
        x=600
        if cv2.contourArea(c)>x:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(frame,("BOLT DETECTED"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                
    cv2.imshow("FRAME",frame)
    delay(3)
    cap.release()
    cam.close()
    cv2.destroyAllWindows()
    return

while True:
    
    x=odometer()
    ultra()
    forward()
    if(ceil(x)==101):
        print('stopping bot')
        print(x)
        stop_bot()
        count = 0
        photo()
GPIO.cleanup()