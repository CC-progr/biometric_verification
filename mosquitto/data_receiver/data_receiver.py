"""Module that allows straighforward subscribing and processing messages"""
from paho.mqtt import subscribe
from pickle import dumps
from datetime import datetime
import os


""" 
Callback function. Function that will be executed for each message received.
The input parameters are the ones specified in the documentation. """
def on_message_save(client, userdata, message):
    print(f"Received message '{message.payload.decode('utf-8')}' on topic '{message.topic}' with QoS {message.qos}")

subscribe.callback(on_message_save, "/HR/sensor1", hostname="mqtt-broker",auth={"username": "user1", "password": "1234"}, port=1883)
subscribe.callback(on_message_save, "/GSR/sensor1", hostname="mqtt-broker",auth={"username": "user1", "password": "1234"}, port=1883)