//////////////////////////////////////////////////////////////////////////////
// TP PID control Grenoble INP-GI
// 2020 - Aurelie Morin
// Sensor : SharpIR sensor GP2Y0A21YK0F on pin A0
// Servomotor : FS5106B with signal on pin 9
//////////////////////////////////////////////////////////////////////////////



#include <Servo.h> // Include Servo library

//////////////////////////////////Variables///////////////////////////////////
int SensorPin = A0; // Distance sensor signal wire on A0 pin (analog input)
int ServoPin = 9; // Servo signal wire on pin 9
Servo myservo;  // Create a new Servo object called myservo
int Read = 0;
float distance = 0.0; // Variable to store the distance
unsigned long currentTime;
unsigned long prevTime;
float distance_previous_error, distance_error;
float samplingTime = 50.;  // Refresh rate period of the loop is 50ms
//////////////////////////////////////////////////////////////////////////////

/////////////////// Offset settings //////////////////////
float sensorOffset = 0.3; // Enter IR sensor offset value here
int servoOffset = 3; // Enter servo motor offset value here
//////////////////////////////////////////////////////////

///////////////////////////PID constants//////////////////////////////////////
float kp = 5; // Proportional gain 
float kd = 2500; // Derivative gain 
float ki = 6; // Integral gain 
float distanceSetpoint = 15.;  // Set the target value of the distance from sensor to the middle of the bar in cm
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;
float PID_total = 0;
//////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);  // Starts serial communication
  myservo.attach(ServoPin);  // Attaches the new servo object called "myservo" to ServoPin
  myservo.write(90+servoOffset); // Set the servo at 90° angle (horizontal position)
  delay(3000); // Wait for 3 seconds
  currentTime = millis();
}


void loop() {
  if (millis() > currentTime+samplingTime)
  {
    currentTime = millis();    
    distance = getDist(100)+sensorOffset;    
   
    Serial.print(distance); // Display curves on the serial plotter
    Serial.print(",");
    Serial.print(40);
    Serial.print(",");
    Serial.print(distanceSetpoint);
    Serial.print(",");
    Serial.println(0);
    
    distance_error = distanceSetpoint - distance; // Error calculation
    
    /////////////////// PID : Proportional term calculation //////////////////////
    PID_p = kp * distance_error;
    //////////////////////////////////////////////////////////////////////////////

    ///////////////////PID : Derivative term calculation /////////////////////////   
    PID_d = kd*((distance_error - distance_previous_error)/samplingTime);
    //////////////////////////////////////////////////////////////////////////////

    /////////////////// PID : Integral term calculation //////////////////////////
    if(-10 < distance_error && distance_error < 10)
    {
      PID_i = PID_i + (ki * distance_error) * samplingTime * 0.001;
    }
    else
    {
      PID_i = 0;
    }
    ////////////////////////////////////////////////////////////////////////////

    /////////////////// PID : Total calculation ////////////////////////////////
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -180, 180, 0, 180); // Map the PID value to a positive range (servo stroke values from 0 to 180 degrees)
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
    ///////////////////////////////////////////////////////////////////////////

    myservo.write(PID_total+servoOffset);  // Sends new angle value to the servomotor
    distance_previous_error = distance_error;
  }
 
}


float getDist(int n) // The "Get distance" function measures the distance based on the average of n values read from the sensor
{
  long sum = 0;
  for(int i = 0; i<n; i++)
  {
    sum=sum+analogRead(SensorPin);
  }  
  float adc = sum/n;
  float distanceCm = 6222.8*(pow(adc,-1.028)); // Characteristic curve of the sensor
  return(distanceCm);
}
