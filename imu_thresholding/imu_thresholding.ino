// #include <Arduino_LSM6DSOX.h>

#include <Wire.h>
#include <SPI.h>
#include <LSM6DSOXSensor.h>

// // Create an instance of the I2C interface
// TwoWire i2c = Wire;

// Create an instance of the LSM6DSOX sensor
LSM6DSOXSensor sensor(&LSM6DS_DEFAULT_SPI,PIN_SPI_SS1);

// Interrupt pin number on Nicla Vision (double-check the pin used for INT1 on your setup)
const int interruptPinNumber = LSM6DS_INT;

// Interrupt pin for the sensor
LSM6DSOX_SensorIntPin_t interruptPin = LSM6DSOX_INT1_PIN;
#define SR 104.0f // Sample rate. Options are: 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333 and 6667 Hz
#define WTM_LV 199 // Watermark threshold level. Max samples in this FIFO configuration is 512 (accel and gyro only).

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    // Wait for the serial connection to be established
  }
Serial.println("starting");
  // Initialize the I2C bus
  LSM6DS_DEFAULT_SPI.begin();

  // Initialize the LSM6DSOX sensor
  if (sensor.begin() != 0) {
    Serial.println("Failed to initialize the sensor!");
    while (1); // Halt execution if sensor initialization fails
  }
  if (sensor.Enable_G() == LSM6DSOX_OK && sensor.Enable_X() == LSM6DSOX_OK) {
    Serial.println("Success enabling accelero and gyro");
  } else {
    Serial.println("Error enabling accelero and gyro");
    abort();
  }

  // Check device id
  uint8_t id;
  sensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("Wrong id for LSM6DSOX sensor. Check that device is plugged");
    abort();
  } else {
    Serial.println("Success checking id for LSM6DSOX sensor");
  }
    // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
  sensor.Set_X_FS(2);
  // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
  sensor.Set_G_FS(250);
  
  // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
  sensor.Set_X_ODR(SR);
  // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
  sensor.Set_G_ODR(SR);
  
 sensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);
  sensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
  // Configure the interrupt pin as input
  pinMode(interruptPinNumber, INPUT);

  // Enable single tap detection on the selected interrupt pin
  if (sensor.Enable_Double_Tap_Detection(interruptPin) != 0) {
    Serial.println("Failed to enable tap detection!");
    while (1); // Halt if tap detection fails
  }
  Serial.println("Sensor initialized and tap detection enabled.");
  delay(3000);
}

void loop() {
  // Check if the interrupt pin is triggered (tap detected)
  if (digitalRead(interruptPinNumber) == HIGH) {
    // Create a structure to hold the event status
    LSM6DSOX_Event_Status_t eventStatus;

    // Get the sensor's event status (including tap detection)
    if (sensor.Get_X_Event_Status(&eventStatus) == 0) {
      // Check if a tap event was detected
      if (eventStatus.DoubleTapStatus) {
        Serial.println("Tap detected!");

        // Clear the interrupt by reading the status
        sensor.Get_X_Event_Status(&eventStatus); // Clear the tap status
      }else{
         uint8_t drdy;
      sensor.Get_X_DRDY_Status(&drdy);
        Serial.print("stat:");
        Serial.println(drdy);
        Serial.println(*((unsigned int*)&eventStatus),16);
      }
    } else {
     
      Serial.println("Failed to get event status.");
    }
  }

  // Small delay to avoid flooding the serial output
  delay(100);
}