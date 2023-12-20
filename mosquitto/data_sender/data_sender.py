from paho.mqtt import publish

while True:
    publish.single(topic="/light/out", payload="HEllo", hostname="mqtt-broker", auth={"username": "user1", "password": "1234"}, port=1883)