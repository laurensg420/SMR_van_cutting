
#include <Servo.h>

Servo myservo; //create servo object

const int trigPin = 9;
const int echoPin = 10;

const int trigPin2 = 5;
const int echoPin2 = 6;

const int servoPin1 = 11;

// defines variables
long duration;
long duration2;
float distance;
float distance2;
char incomingByte;
const int pulloutdeg = 0;
const int measuredeg = 52;

void setup() {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
    myservo.attach(servoPin1);  // attaches the servo on pin 9 to the servo object

    myservo.write(pulloutdeg);

    Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Ultrasonic Sensor 1 (top_sensor)
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if(incomingByte == 'a') {
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
    
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
    
      // Prints the distance on the Serial Monitor
      Serial.print("Top_duration: ");
      Serial.println(duration);
      incomingByte = 0;
    }
    // Ultrasonic Sensor 2 (side_sensor)
    else if(incomingByte == 'b') {
      // Clears the trigPin
      digitalWrite(trigPin2, LOW);
      delayMicroseconds(2);
    
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin2, LOW);
    
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration2= pulseIn(echoPin2, HIGH);
    
    
      // Prints the distance on the Serial Monitor
      Serial.print("Side_duration: ");
      Serial.println(duration2);
      incomingByte = 0;
    }
    else if(incomingByte == 'c') {
      myservo.write(pulloutdeg);              // tell servo to go to position
      delay(500);
      incomingByte = 0;
    }
    else if(incomingByte == 'd') {
      myservo.write(measuredeg);              // tell servo to go to position
      delay(500);
      incomingByte = 0;
    }
  }
  

}
