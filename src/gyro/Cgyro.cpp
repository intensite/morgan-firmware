

// ******************************************************************************************************************
// S.REMILLARD 2019-08-09
// FILE COMMENTED UNTIL JROWBERG PROVIDE A SOLUTION FOR THIS BUG https://github.com/jrowberg/i2cdevlib/issues/468
// ******************************************************************************************************************


// /* ====================================================================
//  * 2019 Stephen Remillard / Francois Paquette
//  * based on the excelent I2Cdev device library code from Jeff Rowberg
//  * available at https://github.com/jrowberg/i2cdevlib
//  ====================================================================== */
// /* ==========================================================================
//  * I2Cdev device library code is placed under the MIT license
//  * Copyright (c) 2012 Jeff Rowberg 
//  * MIT license detaild snipped *
//  =============================================================================*/
// #include "Cgyro.h"
// #include "I2Cdev.h"
// // #include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================
// void Gyro::setupGyro() {
//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//         Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     // initialize device
//     // Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // load and configure the DMP
//     // Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXGyroOffset(X_GYRO_OFFSETS);
//     mpu.setYGyroOffset(Y_GYRO_OFFSETS);
//     mpu.setZGyroOffset(Z_GYRO_OFFSETS);
//     mpu.setXAccelOffset(X_ACCEL_OFFSETS); 
//     mpu.setYAccelOffset(Y_ACCEL_OFFSETS); 
//     mpu.setZAccelOffset(Z_ACCEL_OFFSETS); 

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0) {
//         // Calibration Time: generate offsets and calibrate our MPU6050
//         mpu.CalibrateAccel(6);
//         mpu.CalibrateGyro(6);
//         mpu.PrintActiveOffsets();
//         // turn on the DMP, now that it's ready
//         // Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);
//         mpuIntStatus = mpu.getIntStatus();
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     } else {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }
// }

// // float[] Gyro::getYPR() {
// //     return ypr;
// // }

// void Gyro::ProcessGyroData() {
//     // if programming failed, don't try to do anything
//     if (!dmpReady) return;

//     mpuIntStatus = mpu.getIntStatus();

//     // get current FIFO count
//     fifoCount = mpu.getFIFOCount();
// 	if(fifoCount < packetSize){
// 	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
// 			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
// 	}
//     // check for overflow (this should never happen unless our code is too inefficient)
//     else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
//         // reset so we can continue cleanly
//         mpu.resetFIFO();
//         Serial.println(F("FIFO overflow!"));

//     // otherwise, check for DMP data ready interrupt (this should happen frequently)
//     } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
//         // read a packet from FIFO
//         while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
//             mpu.getFIFOBytes(fifoBuffer, packetSize);
//             // track FIFO count here in case there is > 1 packet available
//             // (this lets us immediately read more without waiting for an interrupt)
//             fifoCount -= packetSize;
//         }
//         #ifdef OUTPUT_READABLE_YAWPITCHROLL
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//             // Serial.print("ypr\t");
//             // Serial.print(ypr[0] * 180/M_PI);
//             // Serial.print("\t");
//             // Serial.print(ypr[1] * 180/M_PI);
//             // Serial.print("\t");
//             // Serial.println(ypr[2] * 180/M_PI);
            
//         #endif

//         #ifdef OUTPUT_READABLE_REALACCEL
//             // display real acceleration, adjusted to remove gravity
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             Serial.print("areal\t");
//             Serial.print(aaReal.x);
//             Serial.print("\t");
//             Serial.print(aaReal.y);
//             Serial.print("\t");
//             Serial.println(aaReal.z);
//         #endif

//         #ifdef OUTPUT_READABLE_WORLDACCEL
//             // display initial world-frame acceleration, adjusted to remove gravity
//             // and rotated based on known orientation from quaternion
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//             Serial.print("aworld\t");
//             Serial.print(aaWorld.x);
//             Serial.print("\t");
//             Serial.print(aaWorld.y);
//             Serial.print("\t");
//             Serial.println(aaWorld.z);
//         #endif
    
        
//     }
// }