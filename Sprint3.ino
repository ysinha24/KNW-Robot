#include <KNWRobot.h>

KNWRobot* robot;
const int M1_PIN = 6;
const int M2_PIN = 5;
const int SV1_PIN = 12;
const int SV2_PIN = 1;
const int IR_PIN = 4;
const int PING_PIN1 = 5;
const int PING_PIN2 = 2;
const int PING_PIN3 = 3;
const int TEMP_PIN = 0;  //A0
const int INCLO_PIN= 15; //A15

double voltage;

void dropArm();
long pingLeft();
long pingCenter();
long pingRight();
void rotateLeft(int);
void rotateRight(int);
void continueUntilWall(long);
double runThermistor();
void driveFast();

void setup() {
  robot = new KNWRobot();

  bool motor1 = robot->setupMotor(1, M1_PIN);
  bool motor2 = robot->setupMotor(2, M2_PIN);

  bool serv1 = robot->setupServo(11, SV1_PIN);
  bool serv2 = robot->setupServo(12, SV2_PIN);

  bool ir = robot->setupIR(21, IR_PIN);
  bool ping1 = robot->setupPing(31, PING_PIN1);
  bool ping2 = robot->setupPing(32, PING_PIN2);
  bool ping3 = robot->setupPing(33, PING_PIN3);

  bool incl = robot->setupIncline(INCLO_PIN);

  bool therm = robot->setupTemp(TEMP_PIN);

  if (!motor1) {
    robot->printLCD("INVALID m1"); delay(2000);
  }
  if (!motor2) {
    robot->printLCD("INVALID m2"); delay(2000);
  }
  if (!serv1) {
    robot->printLCD("INVALID servo"); delay(2000);
  }
  if (!serv2) {
    robot->printLCD("INVALID Cont servo"); delay(2000);
  }
  if (!ir) {
    robot->printLCD("INVALID IR"); delay(2000);
  }
  if (!ping1) {
    robot->printLCD("INVALID PING"); delay(2000);
  }
  if (!ping2) {
    robot->printLCD("INVALID PING"); delay(2000);
  }
  if (!ping3) {
    robot->printLCD("INVALID PING"); delay(2000);
  }
  if (!therm) {
    robot->printLCD("INVALID Therm"); delay(2000);
  }
  if (!incl) {
    robot->printLCD("INVALID Incl"); delay(2000);
  }

  robot->clearLine(0);
  robot->clearLine(1);
  robot->moveCursor(0, 0);
  
  //robot->printLCD("Enter VOLT: ");
  //voltage = robot->getKeypadInput(1);
  //voltage/=10.0;
}

void loop() {
  robot->clearLine(0);
  robot->clearLine(1);
  robot->moveCursor(0, 0);

  robot->printLCD("Enter Quad: ");
  int task = robot->getKeypadInput(1);

  robot->clearLine(0);
  robot->clearLine(1);
  robot->moveCursor(0, 0);
  delay(2000);

  if (task == 'A') {
    // cross bridge

    driveSlow();
    delay(5000);
    robot->pcaStopAll();
    delay(2000);
    
    // turn around
    rotateRight(90);
    delay(2000);
    drive();
    delay(900);
    robot->pcaStopAll();
    delay(2000);
    rotateRight(90);
    delay(2000);
    
    // find ramp
    robot->pca180Servo(11, 60);
    robot->pcaDC2Motors(1, 300, 2, -130);
    
    while (true)
      if (robot->scanIR(21) > 0) {
        delay(2000);                       // to center on ramp
        robot->pcaStopAll();
        break;
      }
    delay(2000);
    // 45 left degree turn
    rotateLeft(50); // compensation

    // climb ramp
    driveFast();
    while(true){
        if (pingCenter() < 40) {
          robot->pcaStopAll();
          break;
      }
    }
    delay(2000);
    // get off ramp
    rotateRight(85);
    
    continueUntilWall(110);
    delay(2000);
    // find soil
    rotateRight(110);
    delay(1000);
    continueUntilWall(15);
    delay(1000);;
    rotateRight(45);                        // to center on soil

    // test soil
    dropArm();


  }
  if (task == 'B') {
    robot->pca180Servo(11, 0);
    // get to zone
    continueUntilWall(60);
    delay(2000);
    rotateRight(90);
    delay(2000);
    continueUntilWall(70);

    robot->pcaDC2Motors(1, 1000, 2, 0);
    delay(1200);  //test right turn 
    
    //continueUntilWall(60);
    
    robot->pcaDC2Motors(1, 1000, 2, 0);
    delay(1000);
      

    // find ir beacon
    driveSlow();
    while (true) {
      if (robot->scanIR(21) > 0)
        break;
      delay(20);
    }

    robot->pcaDC2Motors(1, 50, 2, -550);
    delay(1200);   //test left turn
    
    // drop off
    continueUntilWall(30);
    delay(1000);
    robot->pcaDC2Motors(1, -350, 2, 300);
    delay(1000);
    robot->pcaStopAll();

  }
  if (task == 'C') {
    int count = 0;
    while (true) {
      driveSlow();
      delay(25);

      if (pingRight() > 70)                  // right side
        count++;
      else
        count = 0;                           // for false readings/cracks
      if (count > 8) {                       // to center robot
        robot->pcaStopAll();
        break;
      }
    }
    rotateRight(90);
    delay(3000);
    driveSlow();
    while (true) {
      delay(5);
      if (pingCenter() <= 35) {
        robot->pcaStopAll();
        break;
      }
    }
    delay(3000);
    rotateRight(90);
    delay(3000);
    count = 0;
    while (true) {
      driveSlow();    // test
      delay(10);
      // left side
      if (pingLeft() > 70)
        count++;
      else
        count = 0;                           // for false readings/cracks
      if (count > 1) {                       // to center robot
        robot->pcaStopAll();
        break;
      }
    }
    delay(1500);
    rotateLeft(90);
    delay(1500);

    robot->pca180Servo(11, 45);
    delay(1500);
    driveSlow();
    while (true) {
      int i = robot->scanIR(21);
      char* list = robot->getIR();
      if (i > 0 && list[0] == 'Z') { 
        delay(1000);
        // to center on ramp
        robot->pcaStopAll();
        break;
      }
    }
    delay(2000);
    // 45 left degree turn
    rotateLeft(30);
    delay(2000);
    driveFast();
    delay(2000);
    robot->pcaStopAll();

    // getIncline
    delay(2000);
    robot->printLCD("Ramp angle: ");
    double adc = 1.0 * robot->getIncline();
    double degree = (adc - 323.11)/-5.3681;
    robot->printLCD(degree,1);
    //y = -5.3681x + 323.11
    delay(15000);
  }

  if (task == 'D') {
    // get off ramp
    driveSlow();
    delay(1000);
    robot->pcaStopAll();
    delay(2000);
    // orient towards zone
    for (int i = 0; i < 36; i++) {
      robot->pca180Servo(11, i);
      int numBeacons = robot->scanIR(21);
      char* beacons = robot->getIR();
      if (numBeacons != 0 && beacons[0] == 'G') {
        if (i < 18)
          rotateLeft(90 - i * 5);
        if (i > 18)
          rotateRight(i * 5 - 90);
        break;
      }
    }
    //rotateLeft(60);
    robot->pcaStopAll();
    bool passed = false;
    // move after detecting robot
    while (true){
      delay(250);
      if (pingCenter() < 170) {
        passed = true;
      }
      if (passed && pingCenter() > 200){
        break;
      }
    }
    driveFast();
     // if robot came back
    while (true) {
      if(pingCenter()<10){
        robot->pcaStopAll();
        break;
      }
      delay(10);
    }
    
    delay(2000);
    
    // take temp reading
    dropArm();

    double  temp = runThermistor();
    robot->printLCD(temp, 2);
    robot->printLCD(" C");
  }
  // test instructions and rest servos
  if (task == 0) {
    // reset servos
    robot->pca180Servo(11, 0);
    robot->pca180Servo(12, 0);
  }
  if (task == 11) {
    // test 180 servo
    robot->printLCD("Enter Angle: ");
    int angle = robot->getKeypadInput(1);
    robot->pca180Servo(11, angle - angle * 0.1);
  }
  if (task == 12) {
    // test 180 servo arm
    dropArm();
  }
  if (task == 13) {
    // test ping
    robot->printLCD("l- ");
    robot->printLCD(pingLeft());
    robot->printLCD("   c- ");
    robot->printLCD(pingCenter());     robot->moveCursor(0, 1);
    robot->printLCD("r- ");
    robot->printLCD(pingRight());
  }
  if (task == 14) {
    // test ir
    robot->printLCD("Chars read: ");
    robot->printLCD(robot->scanIR(21));

    robot->moveCursor(0, 1);
    char* characters = robot->getIR();
    robot->printLCD(characters);
  }
  if (task == 15) {
    // test soil conductance
    
  }
  if (task == 16) {
    // test inclinometer
    robot->printLCD("Ramp angle: ");
    double adc = 1.0 * robot->getIncline();
    double degree = (adc - 323.11)/-5.3681;
    robot->printLCD(degree,1);
    //y = -5.3681x + 323.11
    delay(15000);
  }
  if (task == 17) {
    // test thermistor
    double  temp = runThermistor();
    robot->printLCD(temp, 2);
  }
  if (task == 21) {
    // rotate right
    robot->printLCD("Enter degrees");
    int degree = robot->getKeypadInput(1);
    rotateRight(degree);
  }
  if (task == 22) {
    // rotate left
    robot->printLCD("Enter degrees");
    int degree = robot->getKeypadInput(1);
    rotateLeft(degree);
  }
  if (task == 23) {
    // straight
    continueUntilWall(30);
  }
  if (task == 24) {
    //    right 90-turn
    drive();
    delay(2000);
    robot->pcaDC2Motors(1, 1000, 2, 0);
    delay(900);
    robot->pcaStopAll();
  }
  if (task == 25) {
    // left 90 turn
    drive();
    delay(2000);
    robot->pcaDC2Motors(1, 50, 2, -550);
    delay(800);
    robot->pcaStopAll();
  }
  if (task == 26) {
    // 90 degree left turn
    driveFast();
    delay(3000);
    robot->pcaStopAll();
  }
  
  delay(5000);
}

void driveSlow() {
  robot->pcaDC2Motors(1, 290, 2, -160);
}

void driveFast() {
  robot->pcaDC2Motors(1, 410, 2, -300);
}

void dropArm() {
  robot->pca180Servo(12,180);
}

long pingLeft() {
  return robot->getPing(31);
}

long pingCenter() {
  return robot->getPing(32);
}

long pingRight() {
  return robot->getPing(33);
}

void drive() {
  robot->pcaDC2Motors(1, 330, 2, -200); // 1, -150 2, 275
}

void rotateRight(int degree) {
  // rotate left in measurements of 5 degrees
  for (int i = 0; i < degree / 5; i++) {
    robot->pcaDC2Motors(1, 415 , 2, 415);   //test
    delay(15);
    robot->pcaStopAll();
  }
}

void rotateLeft(int degree) {
  // rotate left in measurements of 5 degrees
  for (int i = 0; i < degree / 5; i++) {
    robot->pcaDC2Motors(1, -330, 2, -330);   //test
    delay(15);
    robot->pcaStopAll();
  }
}

void continueUntilWall(long dis) {
  drive();
  while (true) {
    delay(5);
    if (pingCenter() <= dis) {
      robot->pcaStopAll();
      break;
    }
  }
}

double runThermistor() {
  double tempADC = 0;
  for (int i = 0; i < 10; i++) {
    tempADC += robot->getTemp();
    delay(500);
  }
  tempADC /= 10 ;
  double standard = (tempADC - 156.1145) / -2.62454;         // from data
  return standard;
}
