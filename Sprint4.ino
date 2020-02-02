#include <KNWRobot.h>

KNWRobot* robot;
const int M1_PIN = 6;
const int M2_PIN = 5;
const int SV1_PIN = 3;
const int SV2_PIN = 7;
const int IR_PIN = 4;
const int PING_PIN1 = 5;
const int PING_PIN2 = 2;
const int PING_PIN3 = 3;
const int TEMP_PIN = 0;  //A0
const int INCLO_PIN = 15; //A15

// corrections
int voltage;
int driveCorrection = 0;
int rightCorrection = 0;
int leftCorrection = 0;

int rampCorrection1 = 0;
int rampCorrection2 = 0;

// functions
void dropArm();
void drive();
void driveSlow();
void driveFast();
long pingLeft();
long pingCenter();
long pingRight();
void rotateLeft(int);
void rotateRight(int);
bool adjustAlongLeft(long);
bool adjustAlongRight(long);
void continueUntilWall(long);
void continueUntilWallSlow(long);
double runThermistor();


void setup() {
  robot = new KNWRobot();
  //Serial.begin(9600);
  bool motor1 = robot->setupMotor(1, M1_PIN);
  bool motor2 = robot->setupMotor(2, M2_PIN);

  bool serv1 = robot->setupServo(11, SV1_PIN);
  bool serv2 = robot->setupServo(12, SV2_PIN);
  bool serv3 = robot->setupServo(13, 12);
 

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

  robot->printLCD("Enter VOLT: ");
  voltage = robot->getKeypadInput(1);

  switch (voltage) {
    case 138: 
    case 137: 
    case 136: driveCorrection = -10; rightCorrection = -20; leftCorrection = -8; break;
    case 135:
    case 134: driveCorrection = -5; rightCorrection = -15; leftCorrection = -10; break;
    case 133:
    case 132: driveCorrection = -5; rightCorrection = -7; leftCorrection = -10; break;
    case 131: driveCorrection = 0; rightCorrection = -3; leftCorrection = -10; break;
    case 130: driveCorrection = 10; rightCorrection = -3; leftCorrection = -10; break;
    case 129: driveCorrection = 10; rightCorrection = -8; leftCorrection = -10; break;
    case 128: driveCorrection = 10; rightCorrection = -8; leftCorrection = -10; break;
    case 127: driveCorrection = 10; rightCorrection = -9; leftCorrection = -10; break;
    case 126: driveCorrection = 10; rightCorrection = -10; leftCorrection = -10; break;
    case 125: driveCorrection = 10; rightCorrection = -25; leftCorrection = -20; break;
    
    // general cases if voltages above are off
    //high
    case 1: driveCorrection = -5; rightCorrection = -7; leftCorrection = -10; break;
    // mid
    case 2: driveCorrection = 0; rightCorrection = 0; leftCorrection = -10; break;
    // low
    case 3: driveCorrection = 10; rightCorrection = 10; leftCorrection = -20; break;
  }
}

void loop() {
  robot->clearLCD();

  robot->printLCD("Enter Quad: ");
  int task = robot->getKeypadInput(1);

  robot->clearLCD();
  delay(2000);

  if (task == 'A') {
    // cross bridge

    continueUntilWallSlow(30);
    delay(2000);
    //robot->pcaDC2Motors(1, -240, 2, 170);
    //delay(300);
    //robot->pcaStopAll();
    delay(1000);
    // turn around
    rotateRight(90);
    delay(1000);
    drive();
    delay(1000);
    robot->pcaStopAll();
    delay(1000);
    rotateRight(90);
    delay(1000);
    
    // find ramp
    robot->pca180Servo(11, 50 * .9);
    driveSlow();

    while (true) {
      delay(100);
      if (robot->scanIR(21) > 0) {
        delay(1500);                       // to center on ramp
        robot->pcaStopAll();
        break;
      }
      if (pingCenter() < 30) {
        robot->pcaStopAll();
        break;
      }
    }
    delay(1800); 
    // 45 left degree turn
    rotateLeft(60); // compensation
    robot->pcaDC2Motors(1, -240, 2, 170);
    delay(750);
    // climb ramp
    driveRamp();
    delay(2000);
    driveRamp();
    int initR = pingRight();
    int initL = pingLeft();
    int count = 0;
    rampCorrection2 += 60;
    while (true) {

      if (initR - pingRight() > 5){
        rampCorrection2 += 8;
        initR = pingRight();
        }
      if (initL - pingLeft() > 5){
        rampCorrection1 += 8;
        initL = pingLeft();
      }
      if (pingCenter() <= 20) {
        robot->pcaStopAll();
        break;
      }
      rampCorrection2 += 15;
      driveRamp();
      delay(750);
      count++;
      if(count == 10) // 9 * delay sec timer if ping doesn't find ramp
        break;
    }

    delay(2000);
    robot->pcaDC2Motors(1, -240, 2, 170);
    delay(200);
    // get off ramp
    rotateRight(110);
    driveSlow();
    delay(500);
    
    delay(2000);
    // find soil
    driveSlow();
    delay(1000);
    rotateRight(130);
    delay(1000);
    
    robot->pcaStopAll();
    delay(1000);
    continueUntilWall(25);
    delay(1000);                        // to center on soil

    // test soil
    dropArm();
    delay(5000);
    robot->clearLCD();
    double con = runConductivity();
    robot->printLCD(con, 2);
    robot->printLCD("% water");
    delay(10000);
  }




  if (task == 'B') {
    // find ir and get to zone
    driveSlow();
    while (true)
      if (pingRight() > 100)
        break;

    robot->pcaStopAll();
    delay(1000);
    rotateRight(90);
    delay(1000);

    continueUntilWallSlow(45); // 20
    delay(1000);
    rotateLeft(110);
    delay(1000);
    continueUntilWall(55);
    robot->pca180Servo(11, 15*.9);
    robot->pcaDC2Motors(1, 100, 2, -600); //test right turn
    delay(975);
    // line up with wall
    /*
      int min= 9999;
      while(pingRight()<=min){
      min = pingRight();
      delay(500);
      }*/

    // find ir beacon
    driveSlow();
   
    delay(750);
    while (true) {
      if (robot->scanIR(21) > 0)
        break;
      delay(100);
    }
    delay(750);
    robot->pcaDC2Motors(1, 50, 2, -550);
    delay(950);   //test left turn

    // drop off
    continueUntilWallSlow(40);
    delay(1000);
    robot->pcaDC2Motors(1, -350, 2, 300);
    delay(500);
    robot->pcaStopAll();

  }



  
  if (task == 'C') {
    int count = 0;
    long ping = pingRight();
    bool suc = false;
    int sucCount = 0;
    while (true) {
      if (suc) {
        ping = pingRight();
        suc = false;
      }
      driveSlow();
      delay(100);

      if (pingRight() > 50)                  // right side
        count++;
      else
        count = 0;                           // for false readings/cracks
      if (count > 5) {                       // to center robot
        robot->pcaStopAll();
        break;
      }
    }
    rotateRight(90);
    delay(1000);
    continueUntilWall(45);
    delay(1000);
    rotateRight(100);
    delay(1000);
    count = 0;
    ping = pingLeft();
    suc = false;
    sucCount = 0;
    while (true) {
      if (pingLeft() < 30)
        suc = adjustAlongLeft(ping);
      if (suc) {
        ping = pingLeft();
        suc = false;
      }
      driveSlow();
      delay(100);

      if (pingLeft() > 50)                  // right side
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
    driveSlow();
    delay(750);


    robot->pca180Servo(11, 55*.9);
    delay(1300);
    driveSlow();
    while (true) {
      int i = robot->scanIR(21);
      char* list = robot->getIR();
      if (i > 0 && list[0] == 'Z') {
        delay(800);
        // to center on ramp
        robot->pcaStopAll();
        break;
      }
    }
    delay(300);
    // 45 left degree turn
    rotateLeft(25);
    delay(2000);
    driveFast();
    delay(1500);
    robot->pcaStopAll();

    // getIncline
    robot->clearLCD();
    delay(1000);
    robot->printLCD("Ramp angle: ");
    double adc = 1.0 * robot->getIncline();
    for(int i = 0; i < 19;i++){
      adc+=1.0 * robot->getIncline();
    }
    adc/=20;
    double degree = (adc - 646.1) / -4.2807; // 323
    //Serial.println(degree);
    robot->printLCD(degree, 1);
  }




  if (task == 'D') {
    // get off ramp
    driveSlow();
    delay(1800);
    robot->pcaStopAll();
    delay(1000);
    // orient towards zone
    /*
      for (int i = 0; i < 36; i++) {
      robot->pca180Servo(11, i*5*.9);
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
    */
    rotateLeft(50);
    delay(1000);
    driveSlow();
    delay(500);
    robot->pcaStopAll();
    delay(1000);
    
    int min = 9999;
/*
    rotateLeft(90);
    delay(2000);
    continueUntilWallSlow(40);
    delay(1000);
    rotateRight(90);
    delay(1000);
*/

    bool passed = false;
    int timer = 0;
    // move after detecting robot
    while (true) {
      delay(300);
      if (pingCenter() < 80) {
        passed = true;
      }
      if (passed && pingCenter() > 90) {
        delay(1500);                                // !!!!!!!!!!!!!
        break;
      }
    }
    robot->pca180Servo(11, 53 * .9);
    driveFast();
    /*
    // if robot came back
    while (true) {
      int numBeacons = robot->scanIR(21);
      char* beacons = robot->getIR();
      if (numBeacons != 0 && beacons[0] == 'G')
        break;
      if (pingCenter()<10)
        break;
    }
    */
    delay(2250);
    robot->pcaStopAll();
    delay(1000);/*
    robot->pcaDC2Motors(1, 200 + driveCorrection, 2, -130);
    while (robot->scanIR(21) == 0){
      continue;
      delay(100);
    }
    robot->pcaStopAll();
    delay(1000);
    rotateRight(90);
    delay(500);
    */
    if(pingCenter() < 20){
      rotateLeft(90);
      delay(500);
      robot->pcaDC2Motors(1, 190 , 2, -120);
      delay(1500);
      rotateRight(90);
      delay(1000);
      driveSlow();
      delay(1000);
      robot->pcaStopAll();  
    }
    else{  
      rotateLeft(90);
      delay(1000);
      robot->pca180Servo(11, 170 * .9);
      delay(3000);
      while(true){
        robot->pcaDC2Motors(1, 190 , 2, -120);
        if(robot->scanIR(21) > 0)
          break;
      }
      delay(2000);
      rotateRight(80);
      delay(1000);
      driveSlow();
      delay(1000);
      robot->pcaStopAll();
    }

    // take temp reading
    dropArm();
    delay(10000);
    robot->clearLCD();
    double  temp = runThermistor();
    robot->printLCD(temp, 2);
    robot->printLCD(" C");
    delay(15000);
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
    robot->printLCD("  c- ");
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
    dropArm();
    // test soil conductance
    robot->printLCD("% wAtEr: ");
    delay(3000);
    double conduct = runConductivity();
    robot->printLCD(conduct, 2);
    delay(15000);
    robot->pca180Servo(12, 0);
    
  }
  if (task == 16) {
    // test inclinometer
    robot->printLCD("Ramp angle: ");
    double adc = 1.0 * robot->getIncline();
    for(int i = 0; i < 4;i++){
      adc+=1.0 * robot->getIncline();
    }
    adc/=5;
    double degree = (adc - 646.1) / -4.2807; // 323
    //Serial.println(degree);
    robot->clearLCD();
    robot->printLCD(degree, 1);
    
    //y = -4.2807x + 646.1


    delay(15000);
  }
  if (task == 17) {
    dropArm();
    // test thermistor
    delay(7500);
    robot->printLCD("Temp: ");
    double  temp = runThermistor();
    robot->printLCD(temp, 2);
    delay(15000);
  }
  if (task == 18){
    robot->pcaContServoTime(13,90,5000);  
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
    // straight slow
    continueUntilWallSlow(30);
  }
  if (task == 26) {
    // left 90 turn
    drive();
    delay(2000);
    robot->pcaDC2Motors(1, 50, 2, -550);
    delay(800);
    robot->pcaStopAll();
  }
  if (task == 25) {
    // straight Fast
    driveFast();
    delay(3000);
    robot->pcaStopAll();
  }
  if (task == 27) {
    // rotate small
    rotateLeft();
  }
  if (task == 28) {
    // rotate small
    rotateRight();
  }
  if(task == 29){
    while(true){
    }
  }

  delay(5000);
}


void driveRamp() {
  robot->pcaDC2Motors(1, 380 + driveCorrection + rampCorrection1, 2, -310 - rampCorrection2);
}
void driveSlow() {
  robot->pcaDC2Motors(1, 230 + driveCorrection, 2, -160);
}
void drive() {
  robot->pcaDC2Motors(1, 310 + driveCorrection, 2, -220);
}
void driveFast() {
  robot->pcaDC2Motors(1, 380, 2, -310);
}
void dropArm() {
  robot->pca180Servo(12, 200);
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

void rotateRight() {
  robot->pcaDC2Motors(1, 350 + rightCorrection, 2, 0);   //test 415
  delay(10);
  robot->pcaStopAll();
}
void rotateRight(int degree) {
  // rotate left in measurements of 5 degrees
  for (int i = 0; i < degree / 5; i++) {
    robot->pcaDC2Motors(1, 370 + rightCorrection, 2, 370 + rightCorrection);   //test 415
    delay(15);
    robot->pcaStopAll();
  }
}

void rotateLeft() {
  robot->pcaDC2Motors(1, 0, 2, -240 - leftCorrection);   //test 415
  delay(20);
  robot->pcaStopAll();
}
void rotateLeft(int degree) {
  // rotate left in measurements of 5 degrees
  for (int i = 0; i < degree / 5; i++) {
    robot->pcaDC2Motors(1, -340 - leftCorrection, 2, -340 - leftCorrection);   //test 330
    delay(15);
    robot->pcaStopAll();
  }
}

void continueUntilWall(long dis) {
  drive();
  while (true) {
    if (pingCenter() <= dis) {
      robot->pcaStopAll();
      break;
    }
  }
}
void continueUntilWallSlow(long dis) {
  driveSlow();
  while (true) {
    if (pingCenter() <= dis) {
      robot->pcaStopAll();
      break;
    }
    delay(200);
  }
}

bool adjustAlongRight(long initPing) {
  long newPing = pingRight();
  if (newPing > initPing + 5) {
    int min = 9999;
    while (true) {
      rotateRight();
      delay(250);
      int temp = pingRight();
      if (min >= temp)
        min = temp;
      else {
        rotateLeft();
        break;
      }
    }
    return true;
  }
  if (newPing < initPing - 5) {
    int min = 9999;
    while (true) {
      rotateLeft();
      delay(250);
      int temp = pingRight();
      if (min >= temp)
        min = temp;
      else {
        rotateRight();
        break;
      }
    }
    return true;
  }
  return false;
}

bool adjustAlongLeft(long initPing) {
  long newPing = pingLeft();
  if (newPing > initPing + 5) {
    int min = 9999;
    while (true) {
      rotateLeft();
      delay(500);
      int temp = pingLeft();
      if (min >= temp)
        min = temp;
      else {
        rotateRight();
        break;
      }
    }
    return true;
  }
  if (newPing < initPing - 5) {
    int min = 9999;
    while (true) {
      rotateRight();
      delay(500);
      int temp = pingLeft();
      if (min >= temp)
        min = temp;
      else {
        rotateLeft();
        break;
      }
    }
    return true;
  }
  return false;
}

double runConductivity() {
  double conductivity = 0;
  for (int i = 0; i < 5; i++) {
    conductivity += robot->getConductivity();
    delay(250);
  }

  conductivity /= 5;
  // y = -16.735x + 1011
  double standard = (conductivity - 1081) / -16.735;
  return standard;
}

double runThermistor() {
  double tempADC = 0;
  for (int i = 0; i < 50; i++) {
    double x = robot->getTemp();
    tempADC += x;
    delay(50);
  }
  //y = -0.3872x + 67.601


  tempADC /= 50 ;
  //Serial.println(tempADC);
  double standard = tempADC * -0.3872+ 74.601;         // from data1
  //double standard = (tempADC - 110.04) / -0.6805;              //from data 2  
  return standard;
}
