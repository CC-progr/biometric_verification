#include <RUBEN_inferencing.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#if defined(_AVR_ATmega328P) || defined(__AVR_ATmega168_)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; // Infrared LED sensor data
uint16_t redBuffer[100];  // Red LED sensor data

#else
uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100];  // Red LED sensor data

#endif

int32_t bufferLength; // Data length
int32_t spo2; // SPO2 value
int8_t validSPO2; // Indicator to show if the SPO2 calculation is valid
int32_t heartRate; // Heart rate value
int8_t validHeartRate; // Indicator to show if the heart rate calculation is valid

byte pulseLED = 11; // Must be on PWM pin
byte readLED = 13; // Blinks with each data read

// GSR
const int GSR=A2;
int sensorValue;

// PREDICT
static bool debug_nn = false;
const char* highestLabel;

void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 bits per second:
  Serial1.begin(115200);
  while(!Serial);
  Serial.println("Activado Serial");
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; // Wait until user presses a key
  Serial.read();

  byte ledBrightness = 100; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4; // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 1000; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; // Options: 69, 118, 215, 411
  int adcRange = 16384; // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
}

void loop()
{ 
  bufferLength = 100; // Buffer length of 100 stores 4 seconds of samples running at 25sps

  // Read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) // Do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // Calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {

    // Dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //Take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      // Blink onboard LED with every data read
      digitalWrite(readLED, !digitalRead(readLED));

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); 
      // We're finished with this sample so move to next sample
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // TIMESTAMP
    Serial.print("TS: ");
    Serial.println(millis(), DEC);

    // HR
    Serial.print("HR: ");
    Serial.println(heartRate, DEC);
	 
    // GSR
    sensorValue=analogRead(GSR);
    Serial.print("GSR: ");
    Serial.println(sensorValue);

    // PREDICTION
    signal_t signal;
    float sensorValueFloat = static_cast<float>(sensorValue);
    size_t data_size = 1;
    int err = numpy::signal_from_buffer(&sensorValueFloat, data_size, &signal);
    if (err != 0) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
    }
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);

    if (r != EI_IMPULSE_OK)
    {
      ei_printf("ERR: Failed to run classifier (%d)\n", r);
    }
    else
    {
      // Initialize variables for the label and highest score
      uint8_t maxLabel = 0;
      float maxScore = result.classification[0].value;

      // Loop through all classes to find the one with the highest score
      for (uint8_t i = 1; i < 3; i++)
      {
        if (result.classification[i].value > maxScore)
        {
          maxLabel = i;
          maxScore = result.classification[i].value;
        }
      }
      
      highestLabel = result.classification[maxLabel].label;

      Serial.print("PREDICTION: ");
      Serial.println(highestLabel);
      Serial.print("Label 1: ");
      Serial.println(result.classification[0].label);
      Serial.print("Score 1: ");
      Serial.println(result.classification[0].value);
      Serial.print("Label 2: ");
      Serial.println(result.classification[1].label);
      Serial.print("Score 2: ");
      Serial.println(result.classification[1].value);
    }

    Serial.println("************");

    Serial1.write((byte*)&heartRate, sizeof(heartRate));
    delay(500);

    Serial1.write((byte*)&sensorValue, sizeof(sensorValue));
    delay(500);

    Serial1.write(highestLabel);
    delay(500);
  }
}