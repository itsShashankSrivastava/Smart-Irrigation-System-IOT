#!/usr/bin/python
import time
import os
import RPi.GPIO as GPIO
import Adafruit_DHT
from urllib.parse import urlparse
import paho.mqtt.client as paho
import os,sys


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Set sensor type : Options are DHT11,DHT22 or AM2302
sensor=Adafruit_DHT.DHT11

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 11
LCD_D4 = 12
LCD_D5 = 13
LCD_D6 = 15
LCD_D7 = 16
DHT11_Sensor_Pin = 24 #gpio18 in board
RELAY_PIN = 29                
SOIL_SENSOR_PIN = 31
Rain_SENSOR_PIN = 33   




'''
define pin for lcd
'''
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
delay = 1

GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(RELAY_PIN,GPIO.OUT)   # Set pin function as output
GPIO.setup(SOIL_SENSOR_PIN,GPIO.IN)   # Set pin function as input
GPIO.setup(Rain_SENSOR_PIN,GPIO.IN)   # Set pin function as input


def on_connect(self, mosq, obj, rc):
    print("Connected with MQTT Server")
    
def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))
    
mqttc = paho.Client()                        # object declaration
# Assign event callbacks
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.emqx.io:1883') 
url = urlparse(url_str)
mqttc.connect(url.hostname, url.port)


# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

'''
Function Name :lcd_init()
Function Description : this function is used to initialized lcd by sending the different commands
'''
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
'''
Function Name :lcd_byte(bits ,mode)
Fuction Name :the main purpose of this function to convert the byte data into bit and send to lcd port
'''
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
 
  GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
'''
Function Name : lcd_toggle_enable()
Function Description:basically this is used to toggle Enable pin
'''
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
'''
Function Name :lcd_string(message,line)
Function  Description :print the data on lcd 
'''
def lcd_string(message,line):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)


 
# Define delay between readings
delay = 5
lcd_init()
lcd_string("welcome ",LCD_LINE_1)
time.sleep(2)
lcd_string("Smart Irrigation ",LCD_LINE_1)
lcd_string("Project",LCD_LINE_2)
time.sleep(2)
GPIO.output(RELAY_PIN,GPIO.HIGH)  #Relay OFf
motor_staus = 1
while 1:
    rc = mqttc.loop()
    Moisture_output = GPIO.input(SOIL_SENSOR_PIN)
    Rain_output = GPIO.input(Rain_SENSOR_PIN)
    print("Rain_output" + str(Rain_output))
    # Reading the DHT11 is very sensitive to timings and occasionally
    # the Pi might fail to get a valid reading. So check if readings are valid.
    # Use read_retry method. This will retry up to 15 times to
    # get a sensor reading (waiting 2 seconds between each retry).
    humidity, temperature = Adafruit_DHT.read_retry(sensor, DHT11_Sensor_Pin)
    if humidity is not None and temperature is not None:
        lcd_byte(0x01,LCD_CMD) # 000001 Clear display
        lcd_string("Temperature="+ str(temperature),LCD_LINE_1)
        lcd_string("Humidity ="+ str(humidity),LCD_LINE_2)
        mqttc.publish("hum",str(humidity))
        mqttc.publish("Tem",str(temperature))
        time.sleep(1)
        #print('Temp={0:0.1f}*C  Humidity={1:0.1f}% '.format(temperature, humidity))
    else:
        lcd_byte(0x01,LCD_CMD) # 000001 Clear display
        lcd_string("Failed to connect",LCD_LINE_1)
        lcd_string("DHT11 sensor",LCD_LINE_2)
        time.sleep(1)
        break
    
    if(Rain_output == 1):
        if(((temperature > 30) and (humidity < 27)) or (Moisture_output == 1)):
            lcd_byte(0x01,LCD_CMD) # 000001 Clear display
            lcd_string("Motor On",LCD_LINE_2)
            time.sleep(1)
            mqttc.publish("mot","1")
            GPIO.output(RELAY_PIN,GPIO.LOW)  #Relay ON
        else:
            lcd_byte(0x01,LCD_CMD) # 000001 Clear display
            lcd_string("Motor OFF",LCD_LINE_2)
            time.sleep(1)
            mqttc.publish("mot","0")
            GPIO.output(RELAY_PIN,GPIO.HIGH)  #Relay ON
    else:
        GPIO.output(RELAY_PIN,GPIO.HIGH)  #Relay OFf
        mqttc.publish("mot","0")
        #Rain is detected do not start the motor
        lcd_byte(0x01,LCD_CMD) # 000001 Clear display
        lcd_string("Rain Detected",LCD_LINE_1)
        lcd_string("Motor Stop",LCD_LINE_2)
        time.sleep(1)
        
        

  
 
 

