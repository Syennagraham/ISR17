// Delimeter in data output needs to be a # 
//              The assignment
// Time # Pressure Voltage # RPM Value # Pitch # Yaw # Battery Voltage

int avg_rpm; // RPM from sensor A value
int pressure;
int pitch;
int yaw;
int voltage;
int time;
bool doLoop = false;

bool state = false;
int startTime = 0;
int endTime = 0;
int totalTime = 0;

const int buttonPin = 2;      

void pressure_value(){
    pressure = random(0,1023);
    Serial.print(pressure);
  }

  void rpm_value(){   
    avg_rpm = random(0,500);
    Serial.print(avg_rpm);
  }

  void gyro_value() {
    pitch = random(0,180);
    yaw = random(0,180);
    Serial.print(pitch);
    Serial.print("#");
    Serial.print(yaw);
  }

  void battery_voltage(){
    voltage = random(0,12);
    Serial.print(voltage);
  }

  void timeSinceStart(){
    time = millis()/1000;
    Serial.println(time);
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Start serial communication at 9600 for debug statements
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}


void loop() {
  if (digitalRead(buttonPin) &&  state == false) {
    state = true;
    startTime = millis();
    Serial.println("button pressed");

  } 
  else if (state == true && !digitalRead(buttonPin)) {
    state = false;
    endTime = millis();
    totalTime = endTime - startTime;
    if (totalTime >= 1000) {
      if (doLoop) {
        doLoop = false;
      } else {
        doLoop = true;
      }
    }
  }
  
      if (doLoop == true) {
        pressure_value();
        Serial.print("#");
        rpm_value();
        Serial.print("#");
        gyro_value();
        Serial.print("#");
        battery_voltage();
        Serial.print("#");
        timeSinceStart();
        
      }
}
