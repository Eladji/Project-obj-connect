#include <Wire.h>
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_SSD1306.h> // OLED display library
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDRESS 0x3C // SSD1306 I2C address

// Define the MPU6050 address
#define MPU6050_ADDRESS 0x68 // MPU6050 I2C address
const int ledRed = 18;
const int ledGreen = 19;
const int motorL = 17;
int pos;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display object
Adafruit_MPU6050 mpu; // MPU6050 sensor object
Servo servo;
void motorTask(void * parameter) {
  for (;;) {
    enableblade();
    disableblade();
    delay(1000); // delay to prevent the task from using 100% CPU
  }
}

void setup() {
  servo.attach(motorL, 500, 2400);
  Serial.begin(115200);
  Wire.begin(21, 22); // Initialize I2C bus with SDA and SCL pins
  pinMode(21, INPUT_PULLUP); // Enable internal pull-up resistor for SDA pin (GPIO 21)
  pinMode(22, INPUT_PULLUP); // Enable internal pull-up resistor for SCL pin (GPIO 22)
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS); // Initialize with I2C address 0x3C
  display.display(); // Clear the display buffer
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setTextSize(1); // Set text size to 1
  display.setCursor(0,0); // Set cursor position to (0,0)
  display.clearDisplay(); // Clear the display buffer
   // Create a new task that runs on core 1
  xTaskCreatePinnedToCore(
    motorTask,   /* Task function. */
    "MotorTask", /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    NULL,        /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  
   if (!OLED_ADDRESS) {
    Serial.println("Failed to find SSD1306 chip");
    digitalWrite(ledRed, HIGH);
    while (1) {
      delay(10);
    }
  }
  Serial.println("SSD1306 Found!");

  // Initialize the MPU6050 sensor with the adapted address
  if (!mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(ledRed, HIGH);
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
 

  
  if (!motorL) {
    Serial.println("Failed to find servo chip");
    digitalWrite(ledRed, HIGH);
    while (1) {
      delay(10);
    }
  }
  Serial.println("servo Found!");
  digitalWrite(ledGreen, HIGH);
  // Set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  // Set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  // Set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);
}
void enableblade(){
  for (pos = 0; pos <= 180; pos += 1) {
    servo.write(pos);
    delay(15);
  }

}
void disableblade(){
  for (pos = 180; pos >= 0; pos -= 1) {
    servo.write(pos);
    delay(15);
}
}
void loop() {
  // Read accelerometer and gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Clear the display
  display.clearDisplay();
  
  // Display accelerometer data
  display.setCursor(0, 0);
  display.println("Accelerometer (raw):");
  display.println("X: " + String(a.acceleration.x));
  display.println("Y: " + String(a.acceleration.y));
  display.println("Z: " + String(a.acceleration.z));
  
  // Display gyroscope data
  display.println("Gyroscope (raw):");
  display.println("X: " + String(g.gyro.x));
  display.println("Y: " + String(g.gyro.y));
  display.print("Z: " + String(g.gyro.z) + "  ");
  //display.print("Temperature: ");
  display.print(temp.temperature);
  display.println(" degC");
  
  // Display the buffer
  display.display();
  
  // Delay for a short time before updating again
  delay(100);
}
