#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define wifi_ssid "Ruben"
#define wifi_password "123456789"

#define mqtt_server "192.168.43.105"
#define mqtt_port 1883
#define mqtt_user "user1"
#define mqtt_password "1234"

#define out_topic "/eps-L1/location/sensor1"
#define in_led 1

#define echoPin 2 // Echo Pin
#define trigPin 0 // Trigger Pin
 
long duration, distance; // Duration used to calculate distance

WiFiClient espClient;
PubSubClient client;

void setup() {
  Serial.begin(115200);
  pinMode(in_led, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  

  digitalWrite(in_led, LOW); 
  delay(5000);
  digitalWrite(in_led, HIGH);

  setup_wifi();
  client.setClient(espClient);
  client.setServer(mqtt_server, mqtt_port);

  digitalWrite(in_led, LOW);
  delay(5000);
  digitalWrite(in_led, HIGH);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  client.publish(out_topic, String(get_distance()).c_str(), true);
  delay(10000);
}

long get_distance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration/58.2;
  return distance;
}
