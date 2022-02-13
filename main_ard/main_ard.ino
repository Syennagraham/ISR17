/* Pitch Controls (Up and Down)
 * Two buttons which control the position of the elevator fins
 * Controlled with one stepper motor for both fins
 * One button pushes the fins down which will cause the sub to pitch downward
 * One button pushes the fins up which will cause the sub to pitch upwards
 */
/* Yaw Controls (Left and Right)
 * Two buttons which control the position of the Rudder fins
 * Controlled with one stepper motor for both fins
 * One button pushes the fins Left which will cause the sub to yaw leftward
 * One button pushes the fins Right which will cause the sub to yaw rightward
 */
/* Autonomous controls button
 * One button that will switch the sub into autonomous mode and the sub will make course adjustments using the gyro scope
 * The switch turns the mode on and off
 */
// Gyro
// 2 stepper motors (Step Pulse Control and Direction Control)
// Define pins

#include "I2Cdev.h"

#define pitchDirPin 2
#define pitchStepPin 3
#define yawDirPin 4
#define yawStepPin 5
#define pitchUp 6
#define pitchDown 7
#define yawLeft 8
#define yawRight 9

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 1  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 

// Variables
int pd = 250; // Pulse Delay period (Sensativity of motor)

boolean setPitchDir = LOW; // Set Pitch Direction
boolean setYawDir = LOW; // Set Pitch Direction

const int dataINA = 11; //RPM sensor
const int dataINC = 12; //RPM sensor 2
const int dataINB = A1; //Proximity sensor
const int dataIND = A0; //Proximity sensor

unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for refresh of reading

int rpmA; // RPM from sensor A value
int rpmB; // RPM from sensor 2 value
int avg_rpm;
int pressure_voltage;

boolean currentstateA; // Current state of PSA input scan
boolean prevstateA; // State of PSA sensor in previous scan
boolean currentstateB; // Current state of PSA input scan
boolean prevstateB; // State of PSA sensor in previous scan

void setup() {
    // Set up for the debugging serial monitor
    Serial.begin(9600); //Start serial communication at 9600 for debug statements

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    pinMode(pitchDirPin, OUTPUT);
    pinMode(pitchStepPin, OUTPUT);
    pinMode(yawDirPin, OUTPUT);
    pinMode(yawStepPin, OUTPUT);
    
    pinMode(pitchUp, INPUT);
    pinMode(pitchDown, INPUT);
    pinMode(yawLeft, INPUT);
    pinMode(yawRight, INPUT);
    pinMode(dataINB,INPUT);
    pinMode(dataIND,INPUT);

    // Set up for the RPM sensor readings
    pinMode(dataINA,INPUT);    
    pinMode(dataINC,INPUT);
    
    prevmillis = 0;
    prevstateA = LOW;  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    

}


void loop() {
   checkPitchUp();
   checkPitchDown();
   checkYawLeft();
   checkYawRight();
   
   pressure();
   Serial.print("#");
   rpm_value();
   Serial.print("#");
   gyro();
   delay(20);
}


void pressure() {
      int sensorVal=analogRead(dataINB);
      int sensorVal2=analogRead(dataIND);

      pressure_voltage = (sensorVal + sensorVal2) / 2;
      Serial.print(pressure_voltage);
}


void checkPitchUp() {
  boolean mode = digitalRead(pitchUp);
  if (mode == HIGH) {
    setPitchDir = HIGH; //Clockwise
    while (mode == HIGH) {
      digitalWrite(pitchDirPin, setPitchDir);
      digitalWrite(pitchStepPin, HIGH);
      delayMicroseconds(pd);
      digitalWrite(pitchStepPin, LOW);
      delayMicroseconds(pd);
      
      mode = digitalRead(pitchUp);
    }
    return;
  }
  else {
    return;
  }
}


void checkPitchDown() {
  boolean mode = digitalRead(pitchDown);
  if (mode == HIGH) {
    setPitchDir = LOW; //Counter Clockwise
    while (mode == HIGH) {
      digitalWrite(pitchDirPin, setPitchDir);
      digitalWrite(pitchStepPin, HIGH);
      delayMicroseconds(pd);
      digitalWrite(pitchStepPin, LOW);
      delayMicroseconds(pd);
      
      mode = digitalRead(pitchDown);
    }
    return;
  }
  else {
    return;
  }
}


void checkYawLeft() {
  boolean mode = digitalRead(yawLeft);
  if (mode == HIGH) {
    setYawDir = HIGH; //Clockwise
    while (mode == HIGH) {
      digitalWrite(yawDirPin, setYawDir);
      digitalWrite(yawStepPin, HIGH);
      delayMicroseconds(pd);
      digitalWrite(yawStepPin, LOW);
      delayMicroseconds(pd);
      
      mode = digitalRead(yawLeft);
    }
    return;
  }
  else {
    return;
  }
}


void checkYawRight() {
  boolean mode = digitalRead(yawRight);
  if (mode == HIGH) {
    setYawDir = LOW; //Counter Clockwise
    while (mode == HIGH) {
      digitalWrite(yawDirPin, setYawDir);
      digitalWrite(yawStepPin, HIGH);
      delayMicroseconds(pd);
      digitalWrite(yawStepPin, LOW);
      delayMicroseconds(pd);
      
      mode = digitalRead(yawRight);
    }
    return;
  }
  else {
    return;
  }
}


void rpm_value()
{   
   // RPMA Measurement
   currentstateA = digitalRead(dataINA); // Read RPMA sensor state
   if( prevstateA != currentstateA) // If there is change in input
     {
       if( currentstateA == HIGH ) // If input only changes from LOW to HIGH
         {
           duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
           rpmA = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
           prevmillis = micros(); // store time for nect revolution calculation
         }
       else
        {
        rpmA = 0;
        }
     }
 
    prevstateA = currentstateA; // store this scan (prev scan) data for next scan
  
   // RPMB Measurement
   currentstateB = digitalRead(dataINB); // Read PSB sensor state
   if( prevstateB != currentstateB) // If there is change in input
     {
       if( currentstateB == HIGH ) // If input only changes from LOW to HIGH
         {
           duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
           rpmB = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
           prevmillis = micros(); // store time for next revolution calculation
         }
        else
         {
           rpmB = 0;
         }
     }
    prevstateB = currentstateB; // store this scan (prev scan) data for next scan
  
    // Calculating average rpm 
    avg_rpm = (rpmA + rpmB) / 2;
    Serial.print(avg_rpm);
}


void gyro() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("pry\t"); 
            Serial.print(ypr[2] * 180/M_PI); //pitch           
            //Serial.print("#");
            //Serial.print(ypr[1] * 180/M_PI); //roll
            Serial.print("#");
            Serial.println(ypr[0] * 180/M_PI); //yaw
                                
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
  }
}
