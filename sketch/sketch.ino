#include <Wire.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_SSD1306.h> // OLED display library
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDRESS 0x3C // SSD1306 I2C address

// Define the MPU6050 address
#define MPU6050_ADDRESS 0x68 // MPU6050 I2C address
const int ledRed = 18;
const int ledGreen = 19;
const int motorL = 17;
int pos;
int tableau[10];
int screen_state;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display object
Adafruit_MPU6050 mpu;                                                     // MPU6050 sensor object
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
Servo servo;
void motorTask(void *parameter)
{
  bool isBladeEnabled = false;
  unsigned long lastTriggerTime = 0;

  for (;;)
  {
    if (isDifferenceBeyond30())
    {
      unsigned long currentTime = millis();

      if (!isBladeEnabled || (isBladeEnabled && currentTime - lastTriggerTime >= 5000))
      {
        if (!isBladeEnabled)
        {
          enableblade();
          isBladeEnabled = true;
        }
        else
        {
          disableblade();
          isBladeEnabled = false;
        }

        lastTriggerTime = currentTime;
      }
      delay(2000); 
    }

    delay(1000); // delay to prevent the task from using 100% CPU
  }
}

void setup()
{
  servo.attach(motorL, 500, 2400);
  Serial.begin(115200);
  Wire.begin(21, 22);        // Initialize I2C bus with SDA and SCL pins
  pinMode(21, INPUT_PULLUP); // Enable internal pull-up resistor for SDA pin (GPIO 21)
  pinMode(22, INPUT_PULLUP); // Enable internal pull-up resistor for SCL pin (GPIO 22)
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS); // Initialize with I2C address 0x3C
  display.display();                                 // Clear the display buffer
  display.setTextColor(SSD1306_WHITE);               // Set text color to white
  display.setTextSize(1);                            // Set text size to 1
  display.setCursor(0, 0);                           // Set cursor position to (0,0)
  display.clearDisplay();                            // Clear the display buffer
                                                     // Create a new task that runs on core 1
  xTaskCreatePinnedToCore(
      motorTask,   /* Task function. */
      "MotorTask", /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      NULL,        /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */

  if (!OLED_ADDRESS)
  {
    Serial.println("Failed to find SSD1306 chip");
    digitalWrite(ledRed, HIGH);
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("SSD1306 Found!");

  if (!motorL)
  {
    Serial.println("Failed to find servo chip");
    digitalWrite(ledRed, HIGH);
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("servo Found!");

  // Initialize the MPU6050 sensor with the adapted address
  if (!mpu.begin(MPU6050_ADDRESS))
  {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(ledRed, HIGH);
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();

  digitalWrite(ledGreen, HIGH);
  /*
  // Set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
*/
  delay(100);
}
bool isDifferenceBeyond30()
{
  for (int i = 0; i < 10; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (abs(tableau[i] - tableau[j]) >= 1.5)
      {
        return true;
      }
    }
  }
  return false;
}
void enableblade()
{

    servo.write(180);
    
  
}
void disableblade()


    servo.write(50);
    
  
}
/*void screenmanage()
{
  switch (screen_state)
  {
  case constant-expression :
    code 
    break;
  
  default:
    break;
  }
}*/
void tableauVal(int val)
{
  // le 5 c'est la longueure du tableau -1
  for (int i = 9; i > 0; i--)
  {
    if (tableau[i - 1])
    {
      tableau[i] = tableau[i - 1];
    }
  }
  tableau[0] = val;
  ;
}
void loop()
{
  // Read accelerometer and gyroscope data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  tableauVal(int(gyro.gyro.x)); // convert value to integer and call the tableau injection fonction
  Serial.println(temp.temperature);
  Serial.println(",");

  Serial.println(",ax");
  Serial.println(accel.acceleration.x);
  Serial.println("ay");
  Serial.println(accel.acceleration.y);
  Serial.println("az");
  Serial.print(accel.acceleration.z);
  Serial.println(",");
  Serial.println("____________________________");
  Serial.println(",gx");
  Serial.print(gyro.gyro.x);
  Serial.println(",gy");
  Serial.print(gyro.gyro.y);
  Serial.println(",gz");
  Serial.println(gyro.gyro.z);
  Serial.println("____________________________");
  // Clear the display
  display.clearDisplay();

  // Display accelerometer data
  display.setCursor(0, 0);
  display.println("Accelerometer: ");
  display.println("X: " + String(accel.acceleration.x));
  display.println("Y: " + String(accel.acceleration.y));
  display.print("Z: " + String(accel.acceleration.z));

  // Display gyroscope data
  display.println("Gyroscope: ");
  display.println("X: " + String(gyro.gyro.x));
  display.println("Y: " + String(gyro.gyro.y));
  display.print("Z: " + String(gyro.gyro.z) + "  ");
  // display.print("Temperature: ");
  display.print(temp.temperature);
  display.println(" degC");

  // Display the buffer
  display.display();

  // Delay for a short time before updating again
  delay(1000);
}
