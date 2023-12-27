#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define wifi_ssid "Ruben" // "PapitaC"
#define wifi_password "123456789" // "wpgr3523"

#define mqtt_server "192.168.43.248" // rb: "192.168.43.149", ch: "192.168.65.105" 
#define mqtt_port 1883
#define mqtt_user "user1"
#define mqtt_password "1234"

#define out_topic_hr "/user/HR/sensor"
#define out_topic_gsr "/user/GSR/sensor"
#define in_led 1

#define echoPin 2 // Echo Pin
#define trigPin 0 // Trigger Pin
 
long duration, distance; // Duration used to calculate distance

WiFiClient espClient;
PubSubClient client;

void setup() {
  delay(1);
  Serial.begin(115200);
  while(!Serial);
  Serial.print("Hola");
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

  // Read form serial
  while(!Serial.available()); 
  long HR = Serial.read();
  client.publish(out_topic_hr, String(HR).c_str(), true);
  
  while(!Serial.available()); 
  byte GSR[4];
  // long pos = 0;
  // while(Serial.available() && pos < ){
  GSR[0] = Serial.read();
  GSR[1] = Serial.read();
  GSR[2] = Serial.read();
  GSR[3] = Serial.read();
    // pos++;
  //}
  // GSR[pos] = '\0';
  // long GSR = 0;
  // while(Serial.available()) {
  //   GSR += Serial.read();
  // }
 // String GSRString = byteArrToString(GSR, sizeof(GSR));
  client.publish(out_topic_gsr, String((char *)GSR).c_str(), true);
  
  delay(1000);
}
