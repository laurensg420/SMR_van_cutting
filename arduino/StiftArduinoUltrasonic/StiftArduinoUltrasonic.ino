const int trigPin = 9;
const int echoPin = 10;

const int trigPin2 = 5;
const int echoPin2 = 6;

// defines variables
long duration;
long duration2;
char incomingByte;

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin2, INPUT); // Sets the echoPin as an Input


Serial.begin(9600); // Starts the serial communication

}

void loop() {
    // Ultrasonic Sensor 1 (top_sensor)
    if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  }
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
  if(incomingByte == 'b') {
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
    Serial.print("Sid_duration: ");
    Serial.println(duration2);
    incomingByte = 0;
  }
}
