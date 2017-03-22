//#include <NewPing.h>
//
//#define TRIGGER_PIN  29  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PIN     23  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//#define GREEN_LED 45
//#define RED_LED 47
//
////NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//
//void setup() {
//  int distance = 0;
//  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
//  pinMode(RED_LED, OUTPUT);  //Assume that the red and green LEDS are the electromagnets to simulate behaviour
//  pinMode(GREEN_LED, OUTPUT);
//}
//
//void loop() {
//  delay(500)  //let this delay be the delay between sensor readings
//  distance = sonar.ping_cm();
//  if (distance>10){
//    digitalWrite(RED_LED, HIGH);
//    digitalWrite(GREEN_LED, LOW);
//  }
//  else if (0 < distance && distance < 10){
//    digitalWrite(GREEN_LED, HIGH);
//    digitalWrite(RED_LED, LOW);
//  }
//}
