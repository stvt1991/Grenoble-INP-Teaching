//////////////////////////////////////////////////////////////////////////////
// TP PID control Grenoble INP-GI
// 2020 - Aurelie Morin
// Sensor : SharpIR sensor GP2Y0A21YK0F on pin A0
// Servomotor : FS5106B with signal on pin 9
// Sensors calibration program
//////////////////////////////////////////////////////////////////////////////

#include <Servo.h> // Include Servo library

Servo myservo;  // Create a new Servo object called myservo
int SensorPin = A0; // Distance sensor signal wire on A0 pin (analog input)
int ServoPin = 9; // Servo signal wire on pin 9
float distance = 0.0; // Variable to store the distance

/////////////////// Offset settings //////////////////////
float sensorOffset = 0; // IR sensor offset, adjust value here between -1.0 and 1.0 cm
int servoOffset = 3.5; // servo motor offset, adjust value between -5 and 5 degrees
//////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);  // Starts serial communication
  myservo.attach(ServoPin);  // Attaches the new servo object called "myservo" to ServoPin
}

void loop() {
  myservo.write(90+servoOffset); // Set the servo at 90° angle (horizontal position)  
  distance = get_dist(100)+sensorOffset; 
  Serial.println(distance);
  delay(200);
}

float get_dist(int n) // The "Get distance" function measures the distance based on the average of n values read from the sensor
{
  long sum = 0;
  for(int i = 0; i<n; i++)
  {
    sum=sum+analogRead(SensorPin);
  }  
  float adc = sum/n;
  float distance_cm = 6222.8*(pow(adc,-1.028)); // Characteristic curve of the sensor
  return(distance_cm);
}
