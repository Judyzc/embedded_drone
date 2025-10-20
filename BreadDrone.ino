#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include "Adafruit_VL53L0X.h"
#include <utility/imumaths.h>


#define SAMPLERATE_DELAY_MS (1000)

// IMU
#define BNO_ADDR 0x28 // either 0x28 or 0x29, need to be 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_ADDR, &Wire);

// Time of Flight
#define TOF_ADDR 0x29
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

// Pressure
#define BMP_ADDR 0x77
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;


void setup(void) 
{
  Serial.begin(115200);

  while (!Serial) delay(10);

  // Initialize IMU
  Serial.println("Initialize IMU.");
  if(!bno.begin()) {
    Serial.print("Failed to boot BNO055");
  } 
  else {
    /* Display the current temperature */
    // int8_t temp = bno.getTemp();
    // Serial.print("Current Temperature: ");
    // Serial.print(temp);
    // Serial.println(" C");
    // Serial.println("");

    bno.setExtCrystalUse(true);
    Serial.println("Completed initializing IMU.");
  }
  

  // Initialize Time of Flight
  Serial.println("Initialize Time of Flight.");
  if (!tof.begin()) {
    Serial.println("Failed to boot VL53L0X");
  }
  else {
    Serial.println("Completed initializing Time of Flight.");
  }


  // Initialize pressure
  Serial.println("Initialize pressure sensor.");
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    
  }
  else {
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("Completed pressure sensor initialization.");
  }


}

void loop(void) 
{
  sample_imu();

  delay(SAMPLERATE_DELAY_MS);

  sample_tof();

  delay(SAMPLERATE_DELAY_MS);

  sample_pressure();

  delay(SAMPLERATE_DELAY_MS);
}

void sample_imu() {
  Serial.println("SAMPLING IMU *********");
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}

void sample_tof() {
  Serial.println("SAMPLING TOF *********");
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); 
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
}

void sample_pressure() {
  Serial.println("SAMPLING PRESSURE *********");
  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
}


  



