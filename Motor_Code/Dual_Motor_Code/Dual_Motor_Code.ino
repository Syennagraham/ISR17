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

// Define pins
 
#define pitchDirPin 2
#define pitchStepPin 3
#define yawDirPin 4
#define yawStepPin 5
#define pitchUp 6
#define pitchDown 7
#define yawLeft 8
#define yawRight 9
 
// Variables
// Direction variables
boolean setPitchDir = LOW; // Set Pitch Direction
boolean setYawDir = LOW; // Set Yaw Direction

// Button variables
boolean buttonLeftPushed = false;
boolean buttonRightPushed = false;
boolean buttonUpPushed = false;
boolean buttonDownPushed = false;

// Button pushed variables
boolean pitchButtonPushed = false;
boolean yawButtonPushed = false;

/*
 * Pulse timing
 * 
 * Pulse Settings in microseconds (microsBetweenSteps)
 * 1000 = slow
 * 750 = moderate slow
 * 500 = moderate fast
 * 250 = fast
 */
unsigned long microsBetweenSteps = 750; // microseconds
unsigned long currMicros;
unsigned long prevStepMicros = 0;

/*
 * Set pins as output and input
 */
void setup() {
  pinMode (pitchDirPin, OUTPUT);
  pinMode (pitchStepPin, OUTPUT);
  pinMode (yawDirPin, OUTPUT);
  pinMode (yawStepPin, OUTPUT);
  pinMode (pitchUp, INPUT);
  pinMode (pitchDown, INPUT);
  pinMode (yawLeft, INPUT);
  pinMode (yawRight, INPUT);
}

/*
 * Run functions continuously 
 */
void loop() {
  currMicros = micros();
  readButtons();
  setDirections();
  runMotors();
}

/*
 * Read buttons and set pushed to true if pins are HIGH
 */
void readButtons() {
  buttonLeftPushed = false;
  buttonRightPushed = false;
  buttonUpPushed = false;
  buttonDownPushed = false;
  pitchButtonPushed = false;
  yawButtonPushed = false;

  if (digitalRead(yawLeft) == HIGH) {
    buttonLeftPushed = true;
    yawButtonPushed = true;
  }
  if (digitalRead(yawRight) == HIGH) {
    buttonRightPushed = true;
    yawButtonPushed = true;
  }
  if (digitalRead(pitchUp) == HIGH) {
    buttonUpPushed = true;
    pitchButtonPushed = true;
  }
  if (digitalRead(pitchDown) == HIGH) {
    buttonDownPushed = true;
    pitchButtonPushed = true;
  }
}

/*
 * Change directions of motor depending on which button is pushed
 * 
 * WARNING: If right and left or up and down are pressed at the same time
 *          up and left will take precendent 
 */
void setDirections() {
  if (buttonLeftPushed) {
    setYawDir = HIGH; //Clockwise
  }
  else if (buttonRightPushed) {
    setYawDir = LOW; //Counter Clockwise
  }

  if (buttonUpPushed) {
    setPitchDir = HIGH; //Clockwise
  }
  else if (buttonDownPushed) {
    setPitchDir = LOW; //Counter Clockwise
  }
}

/*
 * Run motors for one step depending on buttons pushed
 */
void runMotors() {
  if ((currMicros - prevStepMicros) >= microsBetweenSteps) {
    prevStepMicros = currMicros;

    if (pitchButtonPushed) {
      digitalWrite(pitchDirPin, setPitchDir);
      digitalWrite(pitchStepPin, HIGH);
      digitalWrite(pitchStepPin, LOW);
    }
    if (yawButtonPushed) {
      digitalWrite(yawDirPin, setYawDir);
      digitalWrite(yawStepPin, HIGH);
      digitalWrite(yawStepPin, LOW);
    }
  }
}
