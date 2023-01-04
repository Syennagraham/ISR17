/*
 * This code displays RPM data to the terminal
 * Proximity sensor A 
 *    Blue  - GND
 *    Black - 3
 *    Brown - 5V
 * By: Syenna Graham - syennagraham@vt.edu
 */

const int dataINA = 3; //Proximity sensor input A (PSA)

unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for refresh of reading

int rpm; //RPM to be output to terminal screen 
int rpmA; // RPM from sensor A value

boolean currentstateA; // Current state of PSA input scan
boolean prevstateA; // State of PSA sensor in previous scan


void setup() {
    // Set up for the debugging serial monitor
    Serial.begin(9600); //Start serial communication at 9600 for debug statements
    Serial.println("RPM DATA ");

    // Set up for the RPM sensor readings
    pinMode(dataINA,INPUT);       
    prevmillis = 0;
    prevstateA = LOW;  

}

void loop() {
  rpm_value();
}

void rpm_value()
{     
   // RPMA Measurement
   currentstateA = digitalRead(dataINA); // Read RPMA sensor state
   //Serial.println(currentstateA);
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
    //Serial.println("RPM A .. "); //Uncomment for debugging RPM A
    Serial.println(rpmA);        //Uncomment for debugging RPM A
}
