#include <NewPing.h>
#include <AFMotor.h>

//200 steps = 1 rotation -> 20 cm
#define SONAR_NUM 3      
#define TRIGGER_1  31  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_1     47  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define FRONT_MAGNET 37
#define BACK_MAGNET 41

//sensor 2
#define TRIGGER_2 29
#define ECHO_2 53 

#define TRIGGER_3 26
#define ECHO_3 28

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIGGER_1, ECHO_1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(TRIGGER_2, ECHO_2, MAX_DISTANCE), 
  NewPing(TRIGGER_3, ECHO_3, MAX_DISTANCE)
};


AF_Stepper motor1(200, 1);//param 1 for M1 & M2
AF_Stepper motor2(200, 2);//param 2 for M3 & M4


int distance = 0;
int distance2 = 0;
int distance3 = 0;
int distanceFromFront = 0;

bool distanceWithinTolerances (int distanceA, int distanceB, int lowerBound, int upperBound){
  int distanceDifference = distanceA - distanceB;
  Serial.print(distanceA);
  Serial.print(" ");
  Serial.println(distanceB);
    
  if (abs(distanceDifference) >= lowerBound && abs(distanceDifference) <= upperBound){
    return true;
  }
  return false;
}

void doubleStep (int step, int direction, int style) { 
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

void setup() {
  
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(FRONT_MAGNET, OUTPUT);  //Assume that the red and green LEDS are the electromagnets to simulate behaviour
  pinMode(BACK_MAGNET, OUTPUT);
  motor1.setSpeed(250);  // 5000 = 1000 rpm   
  motor2.setSpeed(250);  // 1000 rpm  
  motor1.release();
  motor2.release();
  delay(1000);
}

void findAndOrientWall (){
  //check for change in sensor distance
  int distanceToWall = 0;
  int newDistance = 0;
  int newDistanceCounter = 0;
  int sensorDistance = 0;

  bool isFacingWall = false;
  bool foundWall = false;
  bool wallDetected = false;

  delay(5000);

   //detect distance to wall
   Serial.println("THERE");
  while (distanceToWall == 0){
    Serial.println("HERE");
        while (sonar[0].ping_cm() < 100) { 
          doubleStep(25, BACKWARD, SINGLE);
             Serial.println(sonar[0].ping_cm());
             doubleStep(25, BACKWARD, SINGLE);
        }
        distanceToWall = sonar[0].ping_cm();
        Serial.println("output initial distance: " + distanceToWall);
  }
      
  //detect wall
  while(!isFacingWall){
    if (!foundWall){
      doubleStep(25, BACKWARD, SINGLE);
      //delay(250);
      sensorDistance = sonar[0].ping_cm();
      
      wallDetected = (distanceWithinTolerances (distanceToWall, sensorDistance, 20, 30));
      
      //check if approaching metal wall
      int frontSensor = sonar[1].ping_cm();
//      if (frontSensor < 30){
//        isFacingWall = true;
//        foundWall = true;
//        distanceToWall = frontSensor;
//        Serial.println("going forward");
//        break;
//      } else 
      if (wallDetected){
          Serial.println("True");
          newDistanceCounter++;
          if (newDistanceCounter > 4){
            foundWall = true;
            sensorDistance = distanceToWall;
          }   
      } else if (!wallDetected) {
          newDistanceCounter --;
          Serial.println("False");
          if (newDistanceCounter < -3){
            newDistanceCounter = 0;
            distanceToWall = sensorDistance;
            Serial.println(distanceToWall);
            //Serial.println("reset Distance to Wall: " + distanceToWall);
          }
      }
    } else {
      Serial.println("TURNING");
     turnLeft (25, SINGLE);
      //delay(250);
      //Serial.println(sonar[2].ping_cm());
      int distance2 = sonar[2].ping_cm();
      if (distanceWithinTolerances (distanceToWall, distance2, 0, 10)) {
        isFacingWall = true;
        Serial.println("FINISHED");
      } 
    }
  }
}

void navigateOverWall(){
  //Get close to the wall
  while(sonar[0].ping_cm()>distanceFromFront || sonar[0].ping_cm()== 0){
    doubleStep(FORWARD, 25, SINGLE);
  }
  //Wall detected
  Serial.println("WALL");
  digitalWrite(FRONT_MAGNET, HIGH);
  digitalWrite(BACK_MAGNET, HIGH);
  motor1.setSpeed(60);
  motor2.setSpeed(60);

  doubleStep(FORWARD, 200, DOUBLE);

  //Get close to floor
  while(sonar[0].ping_cm()>distanceFromFront || sonar[0].ping_cm()== 0){
    doubleStep(FORWARD, 25, DOUBLE);
  }

  //Floor detected
  Serial.println("FLOOR");
  digitalWrite(FRONT_MAGNET, LOW);

  doubleSTEP(FORWARD, 25, SINGLE)
  
  digitalWrite(BACK_MAGNET, LOW);
  motor1.setSpeed(250);
  motor2.setSpeed(250);

  doubleStep(FORWARD, 50, SINGLE);
}

void findAndOrientDropSite (){
  //check for change in sensor distance
  int distanceToSite = 0;
  int newDistance = 0;
  int newDistanceCounter = 0;
  int sensorDistance = 0;

  int holdCounter = 0;

  bool isFacingSite = false;
  bool foundSite = false;

  delay(5000);

  //detect dropsite
  while(!isFacingSite){
    if (!foundSite){
      doubleStep(10, FORWARD, SINGLE);
      sensorDistance = sonar[0].ping_cm();
      if (distanceToSite == 0){
        while (sonar[0].ping_cm() == 0){};
        distanceToSite = sonar[0].ping_cm();
      } else if (!distanceWithinTolerances (distanceToSite, sensorDistance, 10, 20)){
          doubleStep(25, FORWARD, SINGLE);
          while  (holdCounter < 5){
            delay(100);
            holdCounter++;
            
            if (distanceWithinTolerances (sonar[0].ping_cm(), sensorDistance, 10, 20)){
              newDistanceCounter ++;
            }
          }
          if (newDistanceCounter > 4){
            foundSite = true;
            sensorDistance = distanceToSite;
          }
          else {
            newDistanceCounter = 0;
            distanceToSite = sensorDistance;
        }   
      }
    } else {
      turnLeft (25, SINGLE);
      if (distanceWithinTolerances (distanceToSite, sonar[1].ping_cm(), 0, 10)) {
        isFacingSite = true;
      } 
    }
  }
}

void loop() {
  //Start
  int sensorCounter = 0;
  
//  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
//    //delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
//  }


  findAndOrientWall();
  navigateOverWall()
  delay(20000);


//  if (distance>10 || distance<5){
//    digitalWrite(FRONT_MAGNET, HIGH);
//    digitalWrite(BACK_MAGNET, HIGH);
//    doubleStep(200, FORWARD, SINGLE);
//    Serial.println("first if statement");
//    
//    //doubleStep(100, BACKWARD, DOUBLE);
//  }
//  else if (5 < distance && distance < 10){
//    digitalWrite(FRONT_MAGNET, LOW);
//    digitalWrite(BACK_MAGNET, LOW);
//    //doubleStep(0, BRAKE, DOUBLE);
//    Serial.println("second if statement");
//    turnLeft(100, SINGLE);
//  }
}
