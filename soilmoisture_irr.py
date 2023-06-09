import BlynkLib
import Adafruit_DHT
import RPi.GPIO as GPIO
import time

BLYNK_AUTH = '4WsHDzxuQ9490azaGIELuwedCwYzmp7'
blynk = BlynkLib.Blynk(BLYNK_AUTH)

channel = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.IN)

def blynk_connected(ping):
    print('Blynk ready. Ping:', ping)
    
while True:
        humidity,temperature = Adafruit_DHT.read_retry(11, 4)
        print('Temp: {0:0.1f}C Humidity: {1:0.1f} %'.format(temperature, humidity))
        
        if GPIO.input(channel):
            print('no water detected')
        else:
            print('water detected')
            
#         if GPIO.input(17):
#             print('its not raining')
#         else:
#             print('its raining')

def update_blynk():
    blynk.virtual_write(0,temperature)
    blynk.virtual_write(1, humidity)
    
while True:
    try:
        blynk.run()
        update_blynk()
        
    except KeyboardInterrupt:
        break
    
    
        time.sleep(5)
