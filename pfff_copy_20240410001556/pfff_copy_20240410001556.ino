#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo servox;
Servo servoy;

const int servox_pin = 3;
const int servoy_pin = 4;

float gForceX, gForceY, gForceZ; //dexrhsimopoieitai

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ; //dexrhsimopoieitai
float gyroX_cal, gyroY_cal, gyroZ_cal; 
float accelX, accelY, accelZ;
float accelX_cal, accelY_cal, accelZ_cal;
float angleX_cal,angleY_cal, angleZ_cal ;
float angle_pitch, angle_roll;
float accel_pitch, accel_roll;
float angle_roll_acc, angle_pitch_acc; //dexrhsimopoieitai
float angle_pitch_output, angle_roll_output; //dexrhsimopoieitai
int acc_calibration_value = 1000; // Enter the accelerometer calibration value, //dexrhsimopoieitai
float angle_acc;

long loop_timer;
int servoXpos = 90;
int servoYpos = 90;
int count = 0;

MPU6050 sensor;


void setupMPU() {
  // Initialize MPU6050
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B); //leei to sygkekrimeno adress tou register pu thelume (power management settings)
  Wire.write(0b00000000); // Configure MPU6050
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); // register gia thn eyaisthisia toy gyroscopiou
  Wire.write(0x08); // Set gyro sensitivity to +/- 500 degrees/sec
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);//register gia eyaisthisia accelerometer
  Wire.write(0x10); // Set accelerometer sensitivity to +/- 8g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); // h wire library tou arduino kanei copy to data poy zhtame kai mesolayei thn epikoinvnia me to mpu mesw toy I2C bus
 //einai to register gia to accelaration
  Wire.endTransmission(false); 
  
  // Request data from the MPU6050
  Wire.requestFrom(0b1101000, 6); 
  int timeout = 1000; 
  while (Wire.available() < 6 && timeout--) {
    delayMicroseconds(1); 
  }
  
  if (timeout <= 0) {
    Serial.println("Error: No response from MPU6050");
    return; 
  }
  
  accelX = Wire.read() << 8 | Wire.read(); 
  accelY = Wire.read() << 8 | Wire.read(); 
  accelZ = Wire.read() << 8 | Wire.read(); 
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0b1101000, 6);
  int timeout = 1000; //χρονόμετρο
  while (Wire.available() < 6 && timeout--) {
    delayMicroseconds(1); 
  }
  
  if (timeout <= 0) {
    Serial.println("Error: No response from MPU6050");
    return;  }

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}


void setup() {
  Serial.begin(9600);
  servox.attach(servox_pin);
  servoy.attach(servoy_pin);
  Wire.begin();

  Serial.println("Initializing the sensor");
  sensor.initialize();
  Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");

  delay(1000);
  Serial.println("Taking Values from the sensor");
  delay(1000);

  setupMPU();
//GIA TO GYRO
  for (int i = 0; i < 500; i++) {
    recordGyroRegisters();
    gyroX_cal += gyroX;
    gyroY_cal += gyroY;
    gyroZ_cal += gyroZ;
  }
  gyroX_cal /= 500;
  gyroY_cal /= 500;
  gyroZ_cal /= 500;
  Serial.print("gyroX_cal: ");
  Serial.print(gyroX_cal);
  Serial.print("  gyroY_cal: ");
  Serial.print(gyroY_cal);
  Serial.print("  gyroZ_cal: ");
  Serial.println(gyroZ_cal);

//GIA ACCEL
  for (int i = 0; i < 500; i++) {
    recordAccelRegisters();
    accelX_cal += accelX;
    accelY_cal += accelY;
    accelZ_cal += accelZ;
  }
  accelX_cal /= 500;
  accelY_cal /= 500;
  accelZ_cal /= 500;
  Serial.print("accelX_cal: ");
  Serial.print(accelX_cal);
  Serial.print("  accelY_cal: ");
  Serial.print(accelY_cal);
  Serial.print("  accelZ_cal: ");
  Serial.println(accelZ_cal);


  loop_timer = micros(); //diavazei to current time k to apothikeyei sto loop timer.
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();

  gyroX -= gyroX_cal;
  gyroY -= gyroY_cal; 
  gyroZ -= gyroZ_cal;


  accelX -= accelX_cal;
  accelY -= accelY_cal; 
  accelZ -= accelZ_cal;

  angle_pitch += gyroX * 0.000122; //conversion factor. Oloklhrvnei to dω, metatrepei se Δθ, kai to prosthetei
  //sto angle pitch, etsi wste na jeroume kathe stigmh ti prosanatolismo exei. EINAI SYGKEKRIMENH THESH OXI ΔΙΑΣΤΗΜΑ
  //TO GYRO X DEN EINAI DEGRESS/SEC, ALLA ANTIPROSWPEYEI RAW VALUE!!
  angle_roll += gyroY * 0.000122;

  angle_pitch += angle_roll * sin(gyroZ * 0.000002131); //corrections factors epeidh peristrofh ston y k z axona
  //ephreasoun ton x ajona
  angle_roll -= angle_pitch * sin(gyroZ * 0.000002131);

   float accel_pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;
   float accel_roll = angle_pitch * sin(gyroZ * 0.000002131); // Apply correction for gyroZ to roll

  // Apply complementary filter to combine gyro and accelerometer data
  angle_pitch = 0.98 * (angle_pitch + accel_pitch) + 0.02 * accel_pitch; // Adjust alpha values as needed
  angle_roll = 0.98 * (angle_roll + accel_roll) + 0.02 * accel_roll; // Adjust alpha values as needed

  servoXpos = map(angle_roll, 90.00, -90.00, 0, 180);
  servoYpos = map(angle_pitch, -90.00, 90.00, 0, 180);

  count++;
  while (micros() - loop_timer < 8000) {
    if (count == 1) {
      if (servoXpos >= 0 && servoXpos <= 180) {
        servox.write(servoXpos);
      }
    }
    if (count == 2) {
      count = 0;
      if (servoYpos >= 0 && servoYpos <= 180) {
        servoy.write(servoYpos);
      }
    }
  }
  loop_timer += 8000;
}


