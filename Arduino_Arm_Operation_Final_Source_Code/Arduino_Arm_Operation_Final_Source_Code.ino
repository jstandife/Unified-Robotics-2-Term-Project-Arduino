#include <Stepper.h>
#include <Servo.h>

//Serial Communication (Lab_8) Variables
const int ledPin = 13;
const int buffSize = 40;
unsigned int inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
byte coordinates[2];

//stepsPerRevolution for both the x-axis and the y-axis stepsPerRevolution is the value that controls the steppers
double stepsPerRevolutionX = 2048;
double stepsPerRevolutionY = 2048;

//Variables for stepPerRevolution equations
double xValue;//stepsPerRevolution in it's raw form
double yValue;//stepsPerRevolution in it's raw form
bool neg;// is the value negative true or false?

//actuator variables
Stepper xaxisStepper(stepsPerRevolutionX, 8, 10, 9, 11);
Stepper yaxisStepper(stepsPerRevolutionY, 4, 5, 3, 2);
Servo rotationServo;
Servo magnetServo;
int Magnet = 12;

//Serial infomation Variables
double xMax = 158;//camera coordinate system max x
double yMax = 118;// camera coordinate system max y
bool Square = true; //is the shape a square true or false
bool shape; // buffer version of square
double px = 40; // pixel x coordinate
double py = 35; // pixel y coordinate

//Command to help control the stepsPerRevolutions negative values;
void checkSign(double value){
  if ( value < 0)
    neg = true;
  else
    neg = false;   
}

//Command to Lift the Magnet up
void lyftMagnet(){
  Serial.println("Lyft Magnet");
  magnetServo.write(0);
  delay(2000);
}

//Command to Lower the Magnet
void lowerMagnet(){
  Serial.println("Lower Magnet");
  magnetServo.write(180);
  delay(2000);
}

//Command to turn to the correct shape pile based on what type of shape it is 
void turn(){
  rotationServo.attach(6);
  if(Square == true){
    rotationServo.write(98);
    delay(3500);
    rotationServo.attach(0);
  }  
  else if (Square == false){
    rotationServo.write(84);
    delay(3000);
    rotationServo.attach(0);
  }
  delay(2500);
}

//Command to turn back to 90 degrees cenetered in the middle does the exact opposite of what turn did
void turnBack(){
  rotationServo.attach(6);
  if(Square == true){
    rotationServo.write(84);
    delay(2570);
    rotationServo.attach(0);
  }  
  else if (Square == false){
    rotationServo.write(98);
    delay(2570);
    rotationServo.attach(0);
  }
  delay(2500);
}

//Command that turns the magnet on
void pickup(){
  digitalWrite(Magnet, 250);
  delay(2500);
}

//Command that turns the magnet off
void drop(){
  digitalWrite(Magnet, 0);
  delay(2500);
}

//Command that controls the x-axis motion based on where the shape is on the cam's coordinate system
void moveXAxis(){
  double posX = 2 * px; // position of pixel x in inches
  posX = posX - xMax;
  posX = posX * 11;
  posX = posX / 316;
  double factor = 2048 / 1.625; //friction/tenstion factor
  xValue = posX * factor;
  stepsPerRevolutionX = abs(xValue);
  checkSign(xValue);
  if (neg == false)
    xaxisStepper.step(stepsPerRevolutionX);
  else
    xaxisStepper.step(-stepsPerRevolutionX);
  delay(2500);
}

//Command that controls the y-axis motion based on where the shape is on the cam's coordinate system
void moveYAxis(){
  double posY = 2 * py; // position of pixel y in inches
  posY = posY - yMax;
  posY = posY * 17;
  posY = posY / 472;
  double factor = 2048 / 1.95; //friction/tenstion factor
  yValue = posY * factor;
  stepsPerRevolutionY = abs(yValue);
  checkSign(yValue);
  if (neg == false)
    yaxisStepper.step(stepsPerRevolutionY);
  else
    yaxisStepper.step(-stepsPerRevolutionY);
  delay(2500);
}

//Command to move the x-axis and y-axis back to their original position for rotating does the opposite of moveXAxis and moveYAxis respectively
void moveBack(){
  checkSign(xValue);
  if (neg == false)
    xaxisStepper.step(-stepsPerRevolutionX);
  else
    xaxisStepper.step(stepsPerRevolutionX); 
  delay(2500);
  checkSign(yValue);
  if (neg == false)
    yaxisStepper.step(-stepsPerRevolutionY);
  else
    yaxisStepper.step(stepsPerRevolutionY);     
}


//Command to suspend Serial Communication
void sendSuspendCmd(){
  // send the suspend-true command
  Serial.println("<S1>");
}

//Command to enable serial communication
void sendEnableCmd(){
  // send the suspend-false command
  Serial.println("<S0>");
}

//Sends the coordinates it just received back to emgu code to confirm it was right
void sendCoordinatesToPC(){
  // send the point data to the PC
  if(shape == true){
    Serial.print("<P");
    Serial.print("Square: ");
    Serial.print(coordinates[0]);
    Serial.print(",");
    Serial.print(coordinates[1]);
    Serial.println(">");
  }
  else if (shape == false){
    Serial.print("<P");
    Serial.print("Triangle: ");
    Serial.print(coordinates[0]);
    Serial.print(",");
    Serial.print(coordinates[1]);
    Serial.println(">");
  }
}

//Command that receives the shape type as well as the x and y coordinates and stores them in a buffer
void getDataFromPC() {
  // receive data from PC and save it into inputBuffer
  if(Serial.available() > 0) {
    char x = Serial.read();
    // the order of these IF clauses is significant
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      shape = inputBuffer[0];
      coordinates[0] = inputBuffer[1];
      coordinates[1] = inputBuffer[2];  
    }
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }
    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

//Command that sets the serial variables as their associated buffer values for calculations
void setSerialValues(){
  Square = shape;
  px = coordinates[0];
  py = coordinates[1];
}

//Function that setups all the actuators
void setup() {
  rotationServo.attach(6);
  magnetServo.attach(7);
  pinMode(Magnet, OUTPUT);
  //pinMode(led, OUTPUT);
  pinMode(ledPin, OUTPUT);
  xaxisStepper.setSpeed(20);
  yaxisStepper.setSpeed(20);
  Serial.begin(9600);
}

//Function that is called in the hidden main and is looped over and over calls the commands in order of the arm operation process
void loop() {
  getDataFromPC();
  if (newDataFromPC == true){
    sendSuspendCmd();
    digitalWrite(ledPin, HIGH);
    lyftMagnet();
    turnBack();
    setSerialValues();
    moveXAxis();
    moveYAxis();
    lowerMagnet();
    pickup();
    lyftMagnet();
    moveBack();
    turn();
    drop();
    digitalWrite(ledPin, LOW);
    sendEnableCmd();
    sendCoordinatesToPC();
    newDataFromPC = false;
  } 
}
