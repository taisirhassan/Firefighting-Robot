/*
   Name: Taisir Hassan
   Teacher: Mr. Gunby TDR4M1
   Date: February 1st, 2021
   Project: Arduino-based Servo Firefighter Robot
   Description: Robot is able to navigate a maze, and reach the centre of it
   once it reaches the centre, it scans for a flame, and uses its fan
   to douse the flame.
*/
//Include Servo Library From Arduino
#include<Servo.h>

//Global Variables
const int trigPin = 2;
const int echoPin = 3;
const int rightServorMotor = 9;
const int leftServoMotor = 10;
const int servoFanPin = 8;
const int FireSensorPin = 13;
const int temperaturePin = A0;
Servo servoLeft;     // Declares left servo signal
Servo servoRight;    // Declares right servo signal
Servo servoFan;     //Declares Fan servo signal
int sensorValue = 0;

int celsius, delay_t;
int maximumTemperature = 0;
int maximumPosition = 0;

double temperature[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Array set for temperature

void setup() { // Setup the Function
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(FireSensorPin, OUTPUT); // LED (Flame Sensor Pin) set to output
  Serial.begin(9600); // Sets up serial bits to 9600
}

void loop() { // Loop function runs over and over
  Serial.println("Start Robot Simulation");

  servoLeft.attach(leftServoMotor);      // Attach left signal to pin 10
  servoRight.attach(rightServorMotor);   // Attach right signal to pin 9

  //start spinning wheels clockwise (move forward)
  servoLeft.write(80);
  servoRight.write(80);

  //wait for the first wall
  Serial.println("Waiting for the first wall");
  waitForWall();

  //turn right
  Serial.println("Turning right at the first wall");
  servoRight.write(120);  //right wheel spins faster than left wheel counterclockwise
  delay(300);

  Serial.println("Moving Cursor away from the first wall");
  delay(1000);

  //move forward again
  Serial.println("Moving towards the second wall");
  servoLeft.write(80);
  servoRight.write(80);


  //wait for second wall
  Serial.println("Waiting to get to the second wall");
  waitForWall();

  //turn left
  Serial.println("Turning left at the second wall");
  servoLeft.write(120); ////right wheel spins faster than left counterclockwise
  delay(300);

  Serial.println("Move Cursor away from the second wall");
  delay(1000);

  //move forward again
  Serial.println("Move Robot Forward");
  servoLeft.write(80);
  servoRight.write(80);
  delay(100);

  //stops the motor and scans for the flame
  servoLeft.detach(); // Detach command means to stop moving the left and right servo motor
  servoRight.detach();
  Serial.println("Scanning for Flame");

  maximumTemperature = 0;
  for (int n = 0; n < 9; n++) { // For loop states that if int i is 0, i is less than 9, it increases incrementally by one
    //  This formula below converts the temperature sensor value into celsius
    temperature[n] = map(((analogRead(temperaturePin) - 20) * 3.04), 0, 1023, -40, 125);
    Serial.print("Temperature : "); // Print out the temperature value in celsius to serial monitor
    Serial.println(temperature[n]);
    Serial.println("Flame is doused! Thanks for running the program");

    if (temperature[n] > maximumTemperature) { // if int i, from temperatures array list is greater than max temp
      maximumTemperature = temperature[n];    //max temp would equal temp int i from array list
      // Max position would also be set to int i
      maximumPosition = n;
    }

    servoRight.attach(rightServorMotor);
    servoRight.write(80);
    delay(55);
    servoRight.write(93);
  }

  //move right servo to max position then stop
  servoRight.attach(rightServorMotor);
  servoRight.write(106);
  delay((9 - maximumPosition) * 55);
  servoRight.write(93);


  //move both servos forward
  servoLeft.attach(leftServoMotor);
  servoRight.attach(rightServorMotor);
  servoRight.write(80);
  servoLeft.write(80);
  delay(1000);

  //stop servos
  servoRight.write(93);
  servoLeft.write(93);


  servoFan.attach(servoFanPin);
  servoFan.write(0); //turns the fan on
  delay(1000);
  servoFan.detach(); //turns fan off
  delay(1000);

  digitalWrite(FireSensorPin, HIGH); //turns led on (flame found)
  delay(2000);
  digitalWrite(FireSensorPin, LOW); // turns led off(flame is either doused or not near sensor)
  delay(4000);
  while (1) {
  }
}



float ultrasonicSensor() { // inputting in outputs for the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * .0343) / 2; // Formula for outputing the distance from an ultrasonic sensor in cm
  return distance;
}

void waitForWall() { // Void function for robot while waiting for a wall
  float distance = 40; // ultrasonicSensor

  while (distance > 20) {  // If distance is greater than 20, output the distance in the serial monitor
    delay(100);
    distance = ultrasonicSensor();
  }
}


