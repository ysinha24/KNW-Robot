#include <KNWRobot.h>

KNWRobot* robot;
const int M1_PIN = 5;
const int M2_PIN = 10;
const int SV1_PIN = 12;
const int IR_PIN = 4;
const int PING_PIN = 2;
const int TEMP_PIN = 0;

const double motorRatio = 525/600;


void setup() {
  robot = new KNWRobot();
     
  bool motor1 = robot->setupMotor(1,M1_PIN);
  bool motor2 = robot->setupMotor(2,M2_PIN);

  bool serv1 = robot->setupServo(11,SV1_PIN);

  bool ir = robot->setupIR(21, IR_PIN);
  bool ping = robot->setupPing(31,PING_PIN); 
  bool therm = robot->setupTemp(TEMP_PIN);

  if(!motor1){
    robot->printLCD("INVALID m1"); delay(2000);
  }
  if(!motor2){
    robot->printLCD("INVALID m2"); delay(2000);
  }
  if(!serv1){
    robot->printLCD("INVALID servo"); delay(2000);
  }
  if(!ir){
    robot->printLCD("INVALID IR"); delay(2000);
  }
  if(!ping){
    robot->printLCD("INVALID PING"); delay(2000);
  }
  if(!therm){
    robot->printLCD("INVALID Therm"); delay(2000);
  }
}

void loop() {
  robot->clearLine(0);
   robot->clearLine(1);

  robot->moveCursor(0,0);
  robot->printLCD("Enter Task#: ");
  int task = robot->getKeypadInput(1);
  robot->clearLine(0);
  robot->clearLine(1);

  robot->moveCursor(0,0);

  if(task == 1){
    // rotate sirvo
    robot->printLCD("Enter Angle: ");
    int angle = robot->getKeypadInput(1);
    robot->pca180Servo(11, angle);
  }
  if(task == 2){
    // move three meters
    robot->printLCD(task);
    robot->pcaDC2Motors(1, 600*motorRatio, 2, 600); // id1,speed1,id2,speed2        
    delay(4100);                            // sec*1000
    robot->pcaStopAll();
  }
  if(task == 3){
    // drive toward specific beacon
    for(int i=0; i<=18;i++){
      rotateTenLeft();
      int numBeacons = robot->scanIR(21);
      // if find any beacon
      if(numBeacons != 0){                        
         rotateTenLeft();         
         robot->pcaDC2Motors(1, 600*motorRatio, 2, 600);  
         delay(4500);
         robot->pcaStopAll();
         break;
      }
    }
  }
  if(task == 4){
    // create representation of wall
    int dis = 16; // how many meter * dis
    
    for(int i = 0; i< dis;i++){
       robot->pcaDC2Motors(1, 288, 2, 300);    // test
       delay(200);
       long pdis = robot->getPing(31);

       if(pdis < 70)                        //test ping distance
        robot->printLCD('X');
       else
        robot->printLCD('O');
    }
           robot->pcaStopAll();

           delay(20000);

  }
  if(task == 5){
      // test themistor probe
      double tempADC = 0;
      for(int i = 0; i< 10; i++){
        tempADC += robot->getTemp();
        delay(500);
      }
      tempADC/=10 ;
      double standard = (tempADC - 156.1145)/-2.62454;           // from data
      robot->printLCD(standard,2);           // 2 decimal places
      delay(20000);
  }
  if(task == 6){
    // test ping
    robot->printLCD(robot->getPing(31));
          delay(20000);
  }
  if(task == 7){
    // test turn
    for(int i= 0; i<9;i++){
      rotateTenLeft();    
      delay(100);
      }
  }
  if(task == 8){
    // test IR sensor and output
     int charactersReadFromIR = robot->scanIR(21);
     robot->printLCD("Chars read: ");
     robot->printLCD(charactersReadFromIR);
  
     char* IRCharacters = robot->getIR();
     robot->moveCursor(0, 1);
     robot->printLCD(IRCharacters);
     delay(20000);
  }
  if(task == 9){
    // test keypad input
     robot->printLCD("FULL SEND !!!!!!!");
     delay(10000);
  }
  //robot->pca180ServoTime(11, 0, 3000);

}


void rotateTenLeft(){
  // calibrate left 10-degree turn
   robot->pcaDC2Motors(1, 240, 2, -310);   //from data
   delay(90);
   robot->pcaStopAll();
}

void rotateTenRight(){
  // calibrate right 10-degree turn
   robot->pcaDC2Motors(1, -240, 2, 310);   //test
   delay(90);
   robot->pcaStopAll();
}
