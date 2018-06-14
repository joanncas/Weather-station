#!/usr/bin/python
from gpiozero import InputDevice
import Adafruit_BMP.BMP085 as BMP085
import paho.mqtt.client as mqtt
import Adafruit_DHT
import RPi.GPIO as GPIO
import os, urlparse
import time
from time import sleep
no_rain = InputDevice(18)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  #pin that the anemometer is connected to
 
timer = time.time()   # initialise the timer
fast_time = -1        # TODO: future use
# Define event callbacks
def on_connect(client, userdata, flags, rc):
    print("rc: " + str(rc))

def on_message(client, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_publish(client, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(client, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
    print(string)

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

topic = 'test'

# Connect
mqttc.username_pw_set('temperatura', 'clavadosxd')
mqttc.connect('m11.cloudmqtt.com', 17082)

# Start subscribe, with QoS level 0
mqttc.subscribe(topic, 0)

# Publish a message

# Continue the network loop, exit when an error occurs
while True:
	sensor = BMP085.BMP085()
	humidity, temperature = Adafruit_DHT.read_retry(11, 4)
    	tem = '{0:0.1f} C'.format(temperature)
        hum = '{0:0.1f} %'.format(humidity)
	pre = '{0:0.2f} Pa'.format(sensor.read_pressure()) # The local pressure
	alt = '{0:0.2f} m'.format(sensor.read_altitude()) # The current altitude
	sea = '{0:0.2f} Pa'.format(sensor.read_sealevel_pressure())
	mqttc.publish(topic, tem)	
	mqttc.publish(topic, hum)
	mqttc.publish(topic, pre)
	mqttc.publish(topic, alt)
	mqttc.publish(topic, sea)
	if not no_rain.is_active:
        	mqttc.publish(topic, "Esta lloviendo :c")
	else:
		mqttc.publish(topic, "No esta lloviendo c:")

	def wind_ping(channel):
    		global timer
    		global fast_time
    		cur_time = (time.time() - timer)  # time for half a revolution
    		wind_speed = 0.667 / cur_time   # reworked from the datasheet
    		if wind_speed < 200:  # add bounce detection here
        		if cur_time < fast_time:
            			fast_time = cur_time   
        		wind_str = "%10.4f" % wind_speed  # convert the float number to a string
        		wind_speed = wind_speed * 3.6  # apply the multiplier to calculate miles per hour
        		windmph_str = "%10.4f" % wind_speed
        		cur_str = "%10.4f" % cur_time
        		mqttc.publish(topic,"VELOCIDAD DEL VIENTO: " + wind_str + " m/s, " + windmph_str + " Km/h")
        		timer = time.time()  #reset the timer for the next revolution
	time.sleep(30)
