#include <Ultrasonic.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <EEPROM.h>

MPU6050 mpu(Wire);
Ultrasonic rightUlt(2, 3);
Ultrasonic leftUlt(4, 5);
Ultrasonic frontUlt(6, 7);
int RMDF = 8, RMDB = 9, RMS = 10, RMInit = 110;
int LMDF = 13, LMDB = 12, LMS = 11, LMInit = 110 * 0.95;
int cent = 0;
int currentDeg = 0;
int previousDeg = 0;
String path = "LLBRSLLSR",counteredPath="";
void setRightMotor(int value , int dir) {
  if (dir == 0) {
    digitalWrite(RMDF, HIGH);
    digitalWrite(RMDB, LOW);
    analogWrite(RMS, value);
  } else if (dir == 1) {
    digitalWrite(RMDF, LOW);
    digitalWrite(RMDB, HIGH);
    analogWrite(RMS, value);
  }
  else {
    digitalWrite(RMDF, HIGH);
    digitalWrite(RMDB, HIGH);
    analogWrite(RMS, value);
  }
}
void setLeftMotor(int value , int dir) {
  if (dir == 0) {
    digitalWrite(LMDF, HIGH);
    digitalWrite(LMDB, LOW);
    analogWrite(LMS, value);
  } else if (dir == 1) {
    digitalWrite(LMDF, LOW);
    digitalWrite(LMDB, HIGH);
    analogWrite(LMS, value);
  }
  else {
    digitalWrite(LMDF, HIGH);
    digitalWrite(LMDB, HIGH);
    analogWrite(LMS, value);
  }
}
int returnLeft() {
  return  leftUlt.read();
}
int returnRight() {
  return rightUlt.read();
}
int returnFront() {
  return frontUlt.read();
}
void PID(int dir) {
  int P = 0;
  int tem;
  if(dir ==1)
    tem = returnLeft();
  else tem = returnRight();
   
  if (tem > cent) {
    tem = tem % cent;
    P = tem * 9.9;
    setLeftMotor (LMInit *0.85 - P, 1);
    setRightMotor(RMInit *0.85+ P, 1);
  }
  else if (tem < cent)
  {
    tem = cent - tem;
    P = tem * 9.9;
    setLeftMotor(LMInit * 0.85+ P, 1);
    setRightMotor(RMInit *0.85- P, 1);
  } else
  {
    setLeftMotor (LMInit* 0.85, 1);
    setRightMotor(RMInit* 0.85, 1);
  }

}

void stopMotorLeft() {
  setLeftMotor(254, 2);
}

void stopMotorRight() {
  setRightMotor(254, 2);
}


void turnRight() {
  mpu.update();
  int nextD = mpu.getAngleZ() + 84;
  setRightMotor(RMInit * 0.95, 0);
  setLeftMotor(LMInit * 0.95, 1);
  while (nextD > mpu.getAngleZ()) {
    mpu.update();
  }
  stopMotorRight();
  stopMotorLeft();
}
void turnLeft() {
  mpu.update();
  int nextD = mpu.getAngleZ() - 84;
  setRightMotor(RMInit * 0.95, 1);
  setLeftMotor(LMInit * 0.95, 0);
  while (nextD < mpu.getAngleZ()) {
    mpu.update();
  }

}
void applyDelay(){
  unsigned long current = millis();
  while((millis()-current) < 500 );
}
void goRight() {
  mpu.update();
  int nextD = mpu.getAngleZ() - 84;
  turnRight();
  while (nextD < mpu.getAngleZ()) {
    mpu.update();
  }
}
void goLeft() {
  mpu.update();
  int nextD = mpu.getAngleZ() - 84;
  turnLeft();
  stopMotorRight();
  stopMotorLeft();
  applyDelay();
  setLeftMotor (LMInit * 0.80, 1);
  setRightMotor(RMInit * 0.80, 1);
  applyDelay();
  applyDelay();
  while (checkLeft()&& returnLeft()!=0);
  
}
void setup() {   
  Serial.begin(9600);
  Wire.begin();
  EEPROM.write(0,(int)'L');
  pinMode(LMDF, OUTPUT);
  pinMode(LMDB, OUTPUT);
  pinMode(LMS, OUTPUT);
  pinMode(RMDF, OUTPUT);
  pinMode(RMDB, OUTPUT);
  pinMode(RMS, OUTPUT);
  cent = returnLeft();
  byte status = mpu.begin();
  while (status != 0) { } // stop everything if could not connect to MPU6050
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
//  for(int i = 0;i<20;i++)
//    Serial.print((char)EEPROM.read(i)); 
}
bool checkForward() {
  int forward_dis = returnFront();
  if (forward_dis > 23 || forward_dis == 0 ) {
    return true;
  }
  else {
    return false;
  }
}
bool checkLeft() {
  int left_dis = returnLeft();
  if (left_dis > 27 || left_dis == 0 ) {
    return true;
  }
  else {
    return false;
  }
}
bool checkRight() {
  int right_dis = returnRight();
  if (right_dis > 27 || right_dis == 0 ) {
    return true;
  }
  else {
    return false;
  }
}
void reduceDistance() {
  setLeftMotor(LMInit * 0.85, 1);
  setRightMotor(RMInit * 0.85, 1);
  int cur = returnFront();
  while (returnFront() > cur-16);
  stopMotorRight();
  stopMotorLeft();
}
String optimizePath(){
  String temp="" ;
  int ind = 0;
  while(path.indexOf("LBR")!=-1 || path.indexOf("LBS")!=-1 || path.indexOf("RBL")!=-1 || path.indexOf("SBL")!=-1 || path.indexOf("SBS")!=-1
        || path.indexOf("LBL")!=-1){
    temp = path.substring(ind , ind+3);
      if(temp == "LBR"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"B" + path.substring(ind);
      ind = 0;
    }else if(temp =="LBS"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"R" + path.substring(ind);
      ind = 0;
    }else if(temp == "RBL"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"B" + path.substring(ind);
      ind = 0;
    }else if(temp == "SBL"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"R" + path.substring(ind);
      ind = 0;
    }else if(temp == "SBS"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"B" + path.substring(ind);
      ind = 0;
    }else if(temp == "LBL"){
      path.remove(ind,3);
      path =path.substring(0,ind) +"S" + path.substring(ind);
      ind = 0;
    }else {
      ind +=1;
    }
  }
  return path;
}
void applyShortestPath(){
  for(int i = 0;i<counteredPath.length();i++){
    if(counteredPath.charAt(i)=='L'){
      while(checkLeft()==false){
        PID(1);
      }
        if(returnFront()!=0)reduceDistance();
          goLeft();
          counteredPath +="L";
          cent = returnLeft();
    }
    else if(counteredPath.charAt(i)=='R')
    {
      while(checkRight()==false){
        PID(2);
      }
      if(returnFront()!=0)reduceDistance();
      turnRight();
        applyDelay();
        setLeftMotor (LMInit*0.80,1);
        setRightMotor(RMInit*0.80,1);
        applyDelay();
        if(returnRight()!=0)while(checkRight() );
    }
    else if(counteredPath.charAt(i)=='B'){
      turnRight();
      applyDelay();
      turnRight();
      applyDelay();
    }
    else if(counteredPath.charAt(i)=='S'){
      while(checkRight()==false){
        PID(1);
      }
      applyDelay();
      while(checkRight()==true){
        PID(1);
      }
      applyDelay();
    }
  }
}
bool frontT , leftT, rightT;
int coun=0;
void loop() {
  if(path==counteredPath){
    stopMotorLeft();
    stopMotorRight();
    delay(3000);
    counteredPath = optimizePath();
    applyShortestPath();
  }
  else if (checkForward() == true && checkLeft()==false) {
    PID(1);
    if(checkRight() && returnFront()>50)
      {
        EEPROM.write(coun++ , (int)'S');
        counteredPath +="S";
        applyDelay();
        while(checkRight()){
          PID(1);
        }
        applyDelay();
      }
  }
  else if (checkLeft() == true) {
    if(returnFront()!=0)reduceDistance();
    goLeft();
    EEPROM.write(coun++ , (int)'L');
    counteredPath +="L";
    cent = returnLeft();
  }
    else if(checkForward()==false ){
      if(returnFront()!=0)reduceDistance();
      if(checkLeft()==false && checkRight() ==false)
        {
          EEPROM.write(coun++ , (int)'B');          
          counteredPath +="B";
          turnRight();
          applyDelay();
          turnRight();
          applyDelay();

        }
      else {
        
        EEPROM.write(coun++ , (int)'R');
        counteredPath +="R";
        turnRight();
        applyDelay();
        setLeftMotor (LMInit*0.80,1);
        setRightMotor(RMInit*0.80,1);
        applyDelay();
        if(returnRight()!=0)while(checkRight() );
      }
      
      cent = returnLeft();
     }
}
