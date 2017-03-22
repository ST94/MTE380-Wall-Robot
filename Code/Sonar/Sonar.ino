// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  31  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     47  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define GREEN_LED 45
#define RED_LED 47

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  //pinMode(RED_LED, OUTPUT);
  //pinMode(GREEN_LED, OUTPUT);
}

void loop() {
  //digitalWrite(RED_LED, HIGH);
  //digitalWrite(GREEN_LED, HIGH);
  delay(500);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  //digitalWrite(RED_LED, LOW);
  //digitalWrite(GREEN_LED, LOW);
  Serial.println(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  //Serial.println("cm");
  //delay(250);
  
  
}
