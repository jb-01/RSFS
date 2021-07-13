#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

Servo servo1;
Servo servo2;
int correct;
int correct2;
int i;

void setup()
{
    servo1.attach(9);
    servo2.attach(10);
    servo1.write(90);
    servo2.write(100);

    Wire.begin();
    TWBR = 24;
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-5426); //XAccel calibration value (must be manually checked/inputted)
    mpu.setYAccelOffset(-1935); //YAccel calibration value (must be manually checked/inputted)
    mpu.setZAccelOffset(1128);  //ZAccel calibration value (must be manually checked/inputted)
    mpu.setXGyroOffset(-41);    //XGyro calibration value (must be manually checked/inputted)
    mpu.setYGyroOffset(84);     //YGyro calibration value (must be manually checked/inputted)
    mpu.setZGyroOffset(75);     //ZGyro calibration value (must be manually checked/inputted)
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    Serial.begin(115200);

    pinMode(2, INPUT);
}

void loop()
{

    while (fifoCount < packetSize)
    {
        //insert here your code
        fifoCount = mpu.getFIFOCount();
    }

    if (fifoCount == 1024)
    {

        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else
    {

        if (fifoCount % packetSize != 0)
        {

            mpu.resetFIFO();
        }

        else
        {

            while (fifoCount >= packetSize)
            {

                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
            }

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            int yawAngles = ypr[0] * 180 / M_PI;   //yaw angles from Euler angles
            int pitchAngles = ypr[1] * 180 / M_PI; //pitch angles from Euler angles

            i++;
            if (i == 5)
            {
                correct = yawAngles;
                correct2 = pitchAngles;
            }
            else
            {
                yawAngles = yawAngles - correct;
                pitchAngles = pitchAngles - correct2;
            }

            int servo1Value = map(yawAngles, -60, 60, 0, 180);
            int servo2Value = map(pitchAngles, 50, -50, 70, 180);
            int swtichState = digitalRead(2);
            if (swtichState == 1)
            {
                servo1.write(servo1Value);
                servo2.write(servo2Value);
            }
            Serial.println(yawAngles);
            Serial.println(pitchAngles);

            Serial.println(i);
        }
    }
}
