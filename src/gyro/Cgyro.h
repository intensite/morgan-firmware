// #include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h"
#include "../lib/I2Cdev.h"
// #include "helper_3dmath.h"
// #include "../lib/MPU6050_6Axis_MotionApps20.h"
#include "../lib/MPU6050_6Axis_MotionApps_V6_12.h"
#include "../configuration/configuration.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
     #include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_WORLDACCEL


// ********************************************************
// * Calibration constants moved to the config file 
// ********************************************************
// // CALIBRATION CONSTANTS CHANGE TO CALIBRATE
// #define X_GYRO_OFFSETS 5
// #define Y_GYRO_OFFSETS 41
// #define Z_GYRO_OFFSETS 57
// #define X_ACCEL_OFFSETS -1198
// #define Y_ACCEL_OFFSETS 97
// #define Z_ACCEL_OFFSETS 1752
    
// #define SDA 23 //0
// #define SCL 22 //4 

class Gyro {
    private:
        MPU6050 mpu;

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 gyr;        // [x, y, z]            gyro sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector

    public:
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float z_gforce;

        Gyro();
        uint8_t setupGyro();
        void ProcessGyroData();
        // float[] Gyro::getYPR();

};


Gyro::Gyro() {
    devStatus = -1;
};
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
uint8_t Gyro::setupGyro() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        // Wire.begin();
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.begin(SDA, SCL, 400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        // Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    uint8_t retries = 0;

    // verify connection
    Serial.println(F("Testing device connections..."));
    while (!mpu.testConnection() && retries < 5) {
        retries++;
        delay(500);
    }
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    retries = 0;
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    while (devStatus !=0 && retries < 5) {
        devStatus = mpu.dmpInitialize();
        retries++;
        delay(500);
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(_CONF.X_GYRO_OFFSETS);
    mpu.setYGyroOffset(_CONF.Y_GYRO_OFFSETS);
    mpu.setZGyroOffset(_CONF.Z_GYRO_OFFSETS);
    mpu.setXAccelOffset(_CONF.X_ACCEL_OFFSETS); 
    mpu.setYAccelOffset(_CONF.Y_ACCEL_OFFSETS); 
    mpu.setZAccelOffset(_CONF.Z_ACCEL_OFFSETS); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(10);    // Stephen Disabled for now (Seems to be affected when the board is started horizontally)
        mpu.CalibrateGyro(10);     // Stephen Disabled for now (Seems to be affected when the board is started horizontally)
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    return devStatus;
}

// float[] Gyro::getYPR() {
//     return ypr;
// }

void Gyro::ProcessGyroData() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    mpuIntStatus = mpu.getIntStatus();
     

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	if(fifoCount < packetSize){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // read a packet from FIFO
        while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            // clearing the buffer after every read apparently solve the FIFO overflow problem
            // https://arduino.stackexchange.com/a/10309
            //mpu.resetFIFO();  
        }
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //z_gforce = gravity.z;
            
            // Serial.print("Gyro data:\t");
            // Serial.print(gyr.x);
            // Serial.print("\t");
            // Serial.print(gyr.y);
            // Serial.print("\t");
            // Serial.println(gyr.z);
            
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);

            // // Convert ypr from radiant to degree
            ypr[_CONF.YAW_AXIS] = (ypr[_CONF.YAW_AXIS] * 180/M_PI);
            ypr[_CONF.PITCH_AXIS] = (ypr[_CONF.PITCH_AXIS] * 180/M_PI ); 
            ypr[_CONF.ROLL_AXIS] = (ypr[_CONF.ROLL_AXIS] * 180/M_PI);
            
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);

            if(_CONF.DEBUG) {
                Serial.print("euler\t");
                Serial.print(euler[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(euler[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println((euler[2] * 180/M_PI) +90);  // Angle corrected for 90deg mount
            }

            // Convert ypr from radiant to degree 
            // AND FEED THE Euler ANGLES IN THE YAW/PITCH/ROLL ARRAY FOR SIMPLICITY
            ypr[_CONF.YAW_AXIS] = (euler[0] * 180/M_PI);
            ypr[_CONF.PITCH_AXIS] = ((euler[2] * 180/M_PI) +90);
            ypr[_CONF.ROLL_AXIS] = (euler[1] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            // Serial.print(F("areal\t"));
            // Serial.print(aaReal.x);
            // Serial.print(F("\t"));
            // Serial.print(aaReal.y);
            // Serial.print(F("\t"));
            // Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            // Serial.print(F("aworld\t"));
            // Serial.print(aaWorld.x);
            // Serial.print(F("\t"));
            // Serial.print(aaWorld.y);
            // Serial.print(F("\t"));
            // Serial.println(aaWorld.z);
        #endif
    
        
    }
}