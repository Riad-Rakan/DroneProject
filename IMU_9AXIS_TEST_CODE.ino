#include <Wire.h>
#define MPU_ADDR 0x68         // if we had plugged in the add to vcc instead of ground the adress would be 0x69. (MPU-9250 I2C address)
//// Accelerometer registers  ////
#define ACCEL_X_OUT_H 0x3B    // 0x stand for hexadecimal
#define ACCEL_X_OUT_L 0x3C    // ex.: high bit and low bit: high  = 0x1200
#define ACCEL_Y_OUT_H 0x3D    //                            low   = 0x0034
#define ACCEL_Y_OUT_L 0x3E    //                            total = 0x1234
#define ACCEL_Z_OUT_H 0x3F    //                            --------------->  0x1200 | 0x34 = 0x1234
#define ACCEL_Z_OUT_L 0x40
////  Gyroscope registers     ////
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
//// Magnetometer registers   ////
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
#define MAG_CNTL 0x0A
int16_t accelerometer_Bias[3] = {0,0,0}; // Accelerometer bias (X, Y, Z)
int16_t gyroscope_Bias[3] = {0,0,0}; // Gyroscope bias (X, Y, Z)
int16_t magnetometer_Bias[3] = {0,0,0}; // Magnetometer bias (X, Y, Z)

void setup(){
  Serial.begin(9600);       // writes to serial console
  Wire.begin();             // initializes coms between arduino and I2C
  
  //// Initialize MPU-9250    ////
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);         // Power Management register
  Wire.write(0);            // Wake up the MPU-9250
                            // WE ARE WRITING 0 TO THE 0x6B REGISTER TO WAKE UP MPU-9250
  Wire.endTransmission(true);
  
  //// Set accelerometer to ±2g range (high precision) ////
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);    // ACCEL_CONFIG register address
  Wire.write(0x00);    // ±2g range (high precision)
  Wire.endTransmission(true);

  //// Set accelerometer to ±250°/s gyro range (high precision) ////
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG register
  Wire.write(0x00);   // FS_SEL = 0 -> ±250 dps
  Wire.endTransmission(true);

  //// Initialize the magnetometer (AK8963) ////
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x0A);  // Control register for AK8963
  Wire.write(0x01);  // Set magnetometer to measurement mode (continuous)
  Wire.endTransmission(true);

  //// Initialize Interrupt ////
  /*
    Write future function here
    NEW DATA INTERUPT CODE HERE
  */

  calibrateAccelerometer();
  calibrateGyroscope();
  calibrateMagnometer();
}
void loop(){

  // Replace this the "true" for this for loop with "if_tick"
  if(true){
  // Read Accelerometer Data
  float accX = (readSensorData(ACCEL_X_OUT_H) - accelerometer_Bias[0]) / 16384.0; // int -> integer
  float accY = (readSensorData(ACCEL_Y_OUT_H) - accelerometer_Bias[1]) / 16384.0; // 16 -> 16 bit integer 2 bytes (the data we will be getting from the imu)
  float accZ = (readSensorData(ACCEL_Z_OUT_H) - accelerometer_Bias[2] + 16384.0) / 16384.0; // _t -> type
  // Read Gyroscope Data
  float gyroX = (readSensorData(GYRO_XOUT_H) - gyroscope_Bias[0]);    // int -> integer
  float gyroY = (readSensorData(GYRO_YOUT_H) - gyroscope_Bias[1]);    // 16 -> 16 bit integer 2 bytes (the data we will be getting from the imu)
  float gyroZ = (readSensorData(GYRO_ZOUT_H) - gyroscope_Bias[2]);    // _t -> type
  // Read Magnetometer Data
  float magX = (readSensorData(MAG_XOUT_H) - magnetometer_Bias[0]);     // int -> integer
  float magY = (readSensorData(MAG_YOUT_H) - magnetometer_Bias[1]);     // 16 -> 16 bit integer 2 bytes (the data we will be getting from the imu)
  float magZ = (readSensorData(MAG_ZOUT_H) - magnetometer_Bias[2]);     // _t -> type

  print_Accelerometer_data_result(accX,accY,accZ);  
  print_Gyroscopedata_data_result(gyroX,gyroY,gyroZ);
  print_Magnetometer_data_result(magX,magY,magZ);
  delay(500);

  }
}
void print_Accelerometer_data_result(float accX,float accY,float accZ){
  // Print Accelerometer data result
  Serial.print("Accelerometer data X: ");
  Serial.print(accX);
  Serial.print(" | Y: ");
  Serial.print(accY);
  Serial.print(" | Z: ");
  Serial.println(accZ);
}
void print_Gyroscopedata_data_result(float gyroX,float gyroY,float gyroZ){
  // Print Gyroscopedata result
  Serial.print("Gyroscopedata data X: ");
  Serial.print(gyroX);
  Serial.print(" | Y: ");
  Serial.print(gyroY);
  Serial.print(" | Z: ");
  Serial.println(gyroZ);
}
void print_Magnetometer_data_result(float magX,float magY,float magZ){
  // Print Magnetometer data result
  Serial.print("Magnetometer data X: ");
  Serial.print(magX);
  Serial.print(" | Y: ");
  Serial.print(magY);
  Serial.print(" | Z: ");
  Serial.println(magZ);
}
int16_t readSensorData(uint8_t reg) {           // reg -> high register
  Wire.beginTransmission(MPU_ADDR);             // re-establihing communication
  Wire.write(reg);                              // telling the chip the high register we want info from
  Wire.endTransmission(false);                  // ending comunication BUT IF YOU WANTED TO WRITE TO THE RESITER INSEAD, YOU WOULD DO "Wire.write(..)" BEFORE THIS
  Wire.requestFrom(MPU_ADDR, 2, true);          // Request 2 bytes (high and low byte) the chip will always give you the info with this register unless you
                                                //      the same procedure from above and change the high register location you want info from
                                                // Combine the high and low byte to form the 16-bit value
  int16_t value = (Wire.read() << 8) | Wire.read();// the <<8 is like doing 4*10*10, it turns x34 into x3400. the the | is you adding them
  return value;                                 // return that value

}
void calibrateAccelerometer() {
  long accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;
  int samples = 1000;  // Number of samples to collect for calibration

  // Collect data for calibration
  for (int i = 0; i < samples; i++){ 
    accelX_sum += readSensorData(ACCEL_X_OUT_H);
    accelY_sum += readSensorData(ACCEL_Y_OUT_H);
    accelZ_sum += readSensorData(ACCEL_Z_OUT_H);
    delay(5);
  }
  // Calculate average values (biases)
  accelerometer_Bias[0] = accelX_sum / samples;
  accelerometer_Bias[1] = accelY_sum / samples;
  accelerometer_Bias[2] = accelZ_sum / samples;
  /*
  // Print calibration result
  Serial.print("Accelerometer Bias X: ");
  Serial.print(accelerometer_Bias[0]);
  Serial.print(" | Y: ");
  Serial.print(accelerometer_Bias[1]);
  Serial.print(" | Z: ");
  Serial.println(accelerometer_Bias[2]);
  */
  return;
}
void calibrateGyroscope() {
  long sum[3] = {0, 0, 0};
  int samples = 200;

  for (int i = 0; i < samples; i++) {
    sum[0] += readSensorData(GYRO_XOUT_H);
    sum[1] += readSensorData(GYRO_YOUT_H);
    sum[2] += readSensorData(GYRO_ZOUT_H);
    delay(5);
  }

  gyroscope_Bias[0] = sum[0] / samples;
  gyroscope_Bias[1] = sum[1] / samples;
  gyroscope_Bias[2] = sum[2] / samples;
}
void calibrateMagnometer() {
  int16_t mx, my, mz;
  int samples = 500;
  int16_t magMin[3] = {32767, 32767, 32767};
  int16_t magMax[3] = {-32768, -32768, -32768};

  Serial.println("Move the sensor in a figure-8 pattern...");

  for (int i = 0; i < samples; i++) {
    mx = readSensorData(MAG_XOUT_H);
    my = readSensorData(MAG_YOUT_H);
    mz = readSensorData(MAG_ZOUT_H);

    if (mx < magMin[0]) magMin[0] = mx;
    if (my < magMin[1]) magMin[1] = my;
    if (mz < magMin[2]) magMin[2] = mz;

    if (mx > magMax[0]) magMax[0] = mx;
    if (my > magMax[1]) magMax[1] = my;
    if (mz > magMax[2]) magMax[2] = mz;

    delay(50);
  }

  magnetometer_Bias[0] = (magMax[0] + magMin[0]) / 2;
  magnetometer_Bias[1] = (magMax[1] + magMin[1]) / 2;
  magnetometer_Bias[2] = (magMax[2] + magMin[2]) / 2;
}