#include "encoder_driver.h"
#include "motor_driver.h"
#include "commands.h"
#include <PID_v1.h>

typedef struct
{

  double target;
  double currentEncoder;
  double lastEncoder;
  double error;
  double input;
  double output;


}
PIDInfo;
PIDInfo leftInfo, rightInfo;
double wheeldiameter = 0.115;
//double encoderresolution = 1560.0;
double encoderresolution = 7390.0;

double Kp = 2.0, Ki = 5.0, Kd = 0.003;
PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp, Ki, Kd, DIRECT);
double pid_rate = 30.0;
double pidinterval = 1000.0 / pid_rate;
long serialtime;
long nextmotion;
int  moving;

#define BTN_TEST A4
int testMode = 0;
long testDelayStepMillis = 33;
long testTimeCounter;
long testDuration=30000;
long nextTestStep;
int testStep = 0;
int testSpeed;
const int maxTestSpeed = 150;




/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;


#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = AUTO_STOP_INTERVAL;

#define BAUDRATE     115200



void runTest()
{

  if (millis() < nextTestStep)
  {
    
    return;

  }
  
  
  nextTestStep = millis() + testDelayStepMillis;
 

  testTimeCounter+=testDelayStepMillis;
  
  Serial.println(String(testSpeed)+" "+ String(leftInfo.input)+" "+String(rightInfo.input));

   
  testSpeed=(float)maxTestSpeed*sin((float)testTimeCounter/(float)testDuration*2.0f*PI);
  
  
  if (testTimeCounter>testDuration)
  {

      testTimeCounter=0;
      
      testStep+=1;

      if (testStep>1)
      {
        testStep=0;
        
      }
      
    
  }
  
  if (testStep == 0 )
  {


      setTargetTicksPerFrame(testSpeed, 0);


  }

  if (testStep == 1)
  {


      setTargetTicksPerFrame(0,testSpeed);


  }

  
  

  lastMotorCommand = millis();



}

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {

    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;


    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;

    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPIDInfo();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      setTargetTicksPerFrame(arg1, arg2);
      Serial.println("OK");
      break;

    case UPDATE_PID:
      Serial.println("OK");
      break;
    case DISPLAY_PIDS:

      Serial.print("Kp:");
      Serial.print(Kp);
      Serial.print(" Kd:");
      Serial.print(Kd);
      Serial.print(" Ki:");
      Serial.print(Ki);

      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}




void setup()
{

  Serial.begin(BAUDRATE);

  initMotorController();

  pinMode(BTN_TEST, INPUT);
  digitalWrite(BTN_TEST, LOW);


  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(pidinterval);
  leftPID.SetOutputLimits(-255, 255);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);



  testMode = digitalRead(BTN_TEST);

  setSpeedWithWheel(0, 0);



}

void setTargetTicksPerFrame(int left, int right)
{



  if (left == 0 && right == 0)
  {

    setMotorSpeeds(0, 0);
    moving = 0;

  }
  else
  {

    moving = 1;
  }


  leftInfo.target = left;

  rightInfo.target = right;



}

void setSpeedWithWheel(int leftSpeed, int rightSpeed)
{



  if (leftSpeed == 0 && rightSpeed == 0)
  {

    setMotorSpeeds(0, 0);
    moving = 0;


  }
  else
  {

    moving = 1;
  }



  double leftPIDSpeed = double(leftSpeed) / 1000.0 * encoderresolution / wheeldiameter / PI / pid_rate;
  leftInfo.target = leftPIDSpeed;
  double rightPIDSpeed = double(rightSpeed) / 1000.0 * encoderresolution / wheeldiameter / PI / pid_rate;
  rightInfo.target = rightPIDSpeed;

}

void resetPIDInfo()
{


  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;

  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;


}


void loop() {


  if (testMode == HIGH)
  {


    runTest();

  }


  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  if (nextmotion <= millis() && moving == 1)
  {

    leftInfo.currentEncoder = readEncoder(LEFT);
    leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder;


    leftInfo.error = leftInfo.target - leftInfo.input;
    leftPID.Compute();
    leftInfo.lastEncoder = readEncoder(LEFT);


    rightInfo.currentEncoder = readEncoder(RIGHT);
    rightInfo.input = rightInfo.currentEncoder - rightInfo.lastEncoder;

    rightInfo.error = rightInfo.target - rightInfo.input;
    rightPID.Compute();
    rightInfo.lastEncoder = readEncoder(RIGHT);


    setMotorSpeeds(leftInfo.output, rightInfo.output);
    nextmotion = millis() + pidinterval;

  }

  //自动超时停止保护

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {

    setTargetTicksPerFrame(0, 0);

  }

  



}



