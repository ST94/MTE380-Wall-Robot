#include <AFMotor.h>


AF_Stepper motor1(200, 1);//param 1 for M1 & M2
AF_Stepper motor2(200, 2);//param 2 for M3 & M4


void doublestep (int step, int direction, int style) {
  while (step--) {
    motor1.step(1, direction, style); 
    motor2.step(1, direction, style); 
  }
}
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  
  motor1.setSpeed(500);  // 100 rpm   
  motor2.setSpeed(500);  // 1000 rpm  

  //motor1.step(100, FORWARD, SINGLE); 
  //motor2.step(100, FORWARD, SINGLE); 
  motor1.release();
  motor2.release();
  delay(1000);
}


void loop() {
  //BACKWARD will make robot go forward in reality, cuz our config was flipped
  doublestep(200, BACKWARD, DOUBLE);
}



