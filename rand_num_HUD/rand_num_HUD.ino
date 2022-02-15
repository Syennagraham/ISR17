int avg_rpm; // RPM from sensor A value
int pressure;
int pitch;
int yaw;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Start serial communication at 9600 for debug statements

}

void loop() {
   pressure_value();
   Serial.print("#");
   rpm_value();
   Serial.print("#");
   gyro_value();
   delay(20);
}


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
  Serial.println(yaw);
}
