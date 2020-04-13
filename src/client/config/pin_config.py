#-*- coding:UTF-8 -*-

import RPi.GPIO as GPIO
import time
import string
import serial

#Key value definition
run_car  = '1'  # Before key
back_car = '2'  #After button
left_car = '3'  #KEY LEFT
right_car = '4' #KEYRIGHT
stop_car = '0'  #Button stop

#Status value definition
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6

#Cart motor pin definition
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Car button definition
key = 8

#Ultrasonic pin definition
EchoPin = 0
TrigPin = 1

#RGB tricolor lamp pin definition
LED_R = 22
LED_G = 27
LED_B = 24 

#Servo pin definition
ServoPin = 23

#Infrared obstacle avoidance pin definition
AvoidSensorLeft = 12
AvoidSensorRight = 17

#Buzzer pin definition
buzzer = 8

#Fire extinguishing motor pin settings
OutfirePin = 2

#Tracking infrared pin definition
#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #Define the first tracking infrared sensor pin on the left as 3 ports
TrackSensorLeftPin2  =  5   #Define the second tracking infrared sensor pin on the left as 5 ports
TrackSensorRightPin1 =  4   #Define the first tracking infrared sensor pin on the right as 4 ports
TrackSensorRightPin2 =  18  #Define the second tracking infrared sensor pin on the right as 18 ports


#Photosensitive resistor pin definition
LdrSensorLeft = 7
LdrSensorRight = 6

#Set GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning message
GPIO.setwarnings(False)

#Motor pin is initialized to output mode
#Key pin is initialized to input mode
#Ultrasonic, RGB tri-color lights, servo pin initialization
#Infrared obstacle avoidance pin initialization
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    GPIO.setup(LdrSensorLeft,GPIO.IN)
    GPIO.setup(LdrSensorRight,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
    #Set the pwm pin and frequency to 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    #Set the servo frequency and initial duty cycle
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)
	
