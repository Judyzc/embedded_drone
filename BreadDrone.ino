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
uint16_t TOF_ZERO_ALTITUDE;

// Pressure
#define BMP_ADDR 0x77
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
float BMP_ZERO_ALTITUDE;

void setup(void) 
{
  Serial.begin(115200);

  while (!Serial) delay(10);

  // Initialize IMU
  Serial.print("Initialize IMU: ");
  if(!bno.begin()) {
    Serial.print("Failed to boot IMU.");
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
  Serial.print("Initialize Time of Flight: ");
  if (!tof.begin()) {
    Serial.println("Failed to boot Time of Flight.");
  }
  else {
    Serial.println("Completed initializing Time of Flight.");
  }


  // Initialize pressure
  Serial.print("Initializing pressure sensor: ");
  if (!bmp.begin_I2C()) {
    Serial.println("Failed to initialize barometer.");
  }
  else {
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("Completed initializing barometer.");
  }

  calibrate();


}

void calibrate() {
  // calibrate barometer
  float default_altitude = 0;
  int count_success = 0;
  while (count_success < 20) {
    if (! bmp.performReading()) {
      continue;
    }
    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("bmp calibration reading: ");
    Serial.println(alt);

    // BMP seems to get wacky readings at first, filter these out
    // in the lab, typically see 90 - 105 meters
    if (alt > 150 || alt < 80) {
      continue;
    }
    count_success += 1;
    default_altitude += alt;
    delay(100);
  }
  BMP_ZERO_ALTITUDE = (float) default_altitude / count_success;
  Serial.print("BMP_ZERO_ALTITUDE: ");
  Serial.println(BMP_ZERO_ALTITUDE);

  // calibrate TOF
  uint16_t default_tof = 0;
  count_success = 0;
  int count = 0;
  while (count_success < 20 && count < 30) {
    count++;
    VL53L0X_RangingMeasurementData_t measure;
    
    tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      uint16_t tof_alt = measure.RangeMilliMeter;
      Serial.print("tof calibration reading: ");
      Serial.println(tof_alt);

      default_tof += tof_alt;
      count_success += 1;
    } else {
      continue;
    }

    delay(100);
  }
  TOF_ZERO_ALTITUDE = default_tof / count_success;
  Serial.print("TOF_ZERO_ALTITUDE: ");
  Serial.println(TOF_ZERO_ALTITUDE);

  // calibrate IMU



}

void loop(void) 
{
  sample_imu();

  sample_tof();

  sample_pressure();

  Serial.println();
  Serial.println();

  delay(SAMPLERATE_DELAY_MS);
}

void sample_imu() {
  // Serial.println("SAMPLING IMU *********");
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("Roll (Euler Z): ");
  double euler_z_pos = -1 * euler.z();
  Serial.println(euler_z_pos);

  Serial.print("Pitch (Euler Y): ");
  double euler_y_pos = -1 * euler.y();
  Serial.println(euler_y_pos);

  Serial.print("Yaw (Euler X): ");
  double euler_x_pos = euler.x();
  if (euler_x_pos > 180) {
    euler_x_pos = euler_x_pos - 360;
  }
  Serial.println(euler_x_pos);
  
  // Quaternion data
  // imu::Quaternion quat = bno.getQuat();
  // Serial.print("qW: ");
  // Serial.print(quat.w(), 4);
  // Serial.print(" qX: ");
  // Serial.print(quat.x(), 4);
  // Serial.print(" qY: ");
  // Serial.print(quat.y(), 4);
  // Serial.print(" qZ: ");
  // Serial.print(quat.z(), 4);
  // Serial.print("\t\t");
  

  /* Display calibration status for each sensor. */
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);
}

void sample_tof() {
  // Serial.println("SAMPLING TOF *********");
  VL53L0X_RangingMeasurementData_t measure;
    
  tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    uint16_t range = measure.RangeMilliMeter;
    // Serial.print("TOF range (mm): "); 
    // Serial.println(range);

    int tof_diff = (int) range - TOF_ZERO_ALTITUDE;
    Serial.print("TOF Height diff (mm): "); 
    Serial.println(tof_diff);
  } else {
    Serial.println("TOF out of range!");
  }
}

void sample_pressure() {
  // Serial.println("SAMPLING PRESSURE *********");
  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  // Serial.print("Temperature = ");
  // Serial.print(bmp.temperature);
  // Serial.println(" *C");

  // Serial.print("Pressure = ");
  // Serial.print(bmp.pressure / 100.0);
  // Serial.println(" hPa");

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  // Serial.print("Barometer Altitude = ");
  // Serial.print(altitude);
  // Serial.println(" m");
  
  float altitude_diff = altitude - BMP_ZERO_ALTITUDE;
  Serial.print("Barometer Altitude Change = ");
  Serial.print(altitude_diff);
  Serial.println(" m");

}


  



