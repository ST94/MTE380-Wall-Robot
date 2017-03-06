#include <NewPing.h>
#include <AFMotor.h>


#define TRIGGER_PIN  29  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     23  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define GREEN_LED 47
#define RED_LED 45
AF_Stepper motor1(200, 1);//param 1 for M1 & M2
AF_Stepper motor2(200, 2);//param 2 for M3 & M4

void doubleStep (int step, int direction, int style) {
  motor1.setSpeed(250);  // 5000 = 1000 rpm   
  motor2.setSpeed(250);  // 1000 rpm  
  while (step--) {
    motor1.step(1, direction, style); 
    motor2.step(1, direction, style); 
  }
}
void turnRight (int step, int style) {
  motor1.setSpeed(250);  // 5000 = 1000 rpm   
  motor2.setSpeed(250);  // 1000 rpm  
  while (step--) {
    motor1.step(1, FORWARD, style); 
    motor2.step(1, BACKWARD, style); 
  }
}

void turnLeft (int step, int style) {
  motor1.setSpeed(100);  // 5000 = 1000 rpm   
  motor2.setSpeed(100);  // 1000 rpm  
  while (step--) {
    motor1.step(1, BACKWARD, style); 
    motor2.step(1, FORWARD, style); 
  }
}


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int distance;
void setup() {
  int distance = 0;
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(RED_LED, OUTPUT);  //Assume that the red and green LEDS are the electromagnets to simulate behaviour
  pinMode(GREEN_LED, OUTPUT);
  motor1.setSpeed(250);  // 5000 = 1000 rpm   
  motor2.setSpeed(250);  // 1000 rpm  

  //motor1.step(100, FORWARD, SINGLE); 
  //motor2.step(100, FORWARD, SINGLE); 
  motor1.release();
  motor2.release();
  delay(1000);
}

void loop() {
  //delay(250);  //let this delay be the delay between sensor readings
  
  distance = sonar.ping_cm();
  Serial.println(distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
  if (distance>10 || distance<5){
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    //doubleStep(100, BACKWARD, DOUBLE);
  }
  else if (5 < distance && distance < 10){
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    //doubleStep(0, BRAKE, DOUBLE);
    turnLeft(100, SINGLE);
  }
}
