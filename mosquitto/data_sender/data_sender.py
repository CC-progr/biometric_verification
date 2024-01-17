from paho.mqtt import publish
import time

while True:
    publish.single(topic="/HR/test", payload="HEllo", hostname="mqtt-broker", auth={"username": "user1", "password": "1234"}, port=1883)
    time.sleep(2)