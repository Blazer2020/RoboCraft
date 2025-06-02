#include <util/atomic.h>
#include <LinkedList.h>
#include <BeeLineSensorPro.h>
#include <Servo.h>

Servo myservo;
Servo trapdoor;

int dul=0;
int countP;
int countPP;
int t1,t4,t6;
int counttask2 = 0; int countsix = 0; int countsix2 = 0; int countsix3 = 0;
int counttask2task2 = 0;
int p = 0,count =0,q=0,r=0,a=0;
unsigned long linetime = 1;
LinkedList<int> barcode;
int startingturn=0;
int start_pos,end_pos,difference,number;
int speed = 100;// Update based on your connection
const int echoPin = 48;
const int trigPin = 49;   // Update based on your connection
long duration;
float distance;
bool gate;
int num;
bool order = false;

int task3=0;
int task4=0;
int leftlinefollowing=0;
int rightlinefollowing=0;
int task4linecount=0;
int stopcount=0;
int starttask=0;

int linefollowingcount=0;
int linefollowingcount2=0;
int allblack=0;
int allblack2=0;
int blackcount=0;
int task7_1=0;
int task7_2=0;
int whitecount=0;
int black=0;

unsigned long dstart, dend, ddifference,dtime=0;

bool height1, height2, height3;
int Height = 0 ,  Height2 = 0, Height3 = 0,Height8;
float gap;

const int trigPin1 = 51;
const int echoPin1 = 50;
const int trigPin2 = 53;
const int echoPin2 = 52;
const int trigPin3 = 47;   // New ultrasonic sensor for height 3
const int echoPin3 = 46;   // New ultrasonic sensor for height 3


//ColorLineFollowing
float kP=0.03; //kp=0.05-Black //white 0.008
float kD=0;
float last_value=0;
BeeLineSensorPro sensor = BeeLineSensorPro((unsigned char[]){ A0,A1,A2,A3,A4,A5,A6,A7},LINE_WHITE);
int s0,s1,s2,s3,s4,s5,s6,s7;
#define M1 7
#define M1pwm 6
#define M2 9
#define M2pwm 8
int i;
int colorlinecorrection=0;

// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0.025), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = speed;
    }
  
    // motor direction
    dir = -1;
    if(u<0){
      dir = 1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {2,18};
const int encb[] = {3,19};
//const int pwm[] = {9,13};
const int in1[] = {M2pwm,M1pwm};
const int in2[] = {M2,M1};

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M1pwm, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M2pwm, OUTPUT);
  pinMode(13,OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    //pinMode(pwm[k],OUTPUT);

    pid[k].setParams(1,0,0,255);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  pinMode(22,INPUT);pinMode(23,INPUT);pinMode(24,INPUT);pinMode(25,INPUT);pinMode(26,INPUT);pinMode(27,INPUT);pinMode(28,INPUT);pinMode(29,INPUT);
  //Serial.println("target pos");


  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(30, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(32,INPUT_PULLUP);
  pinMode(34,INPUT_PULLUP);
  pinMode(37,INPUT_PULLUP);
  pinMode(38,INPUT_PULLUP);
  pinMode(40,INPUT_PULLUP);  

  delay(5000);
  //digitalWrite(13,HIGH);


  myservo.attach(12);  
  myservo.write(30);
  trapdoor.attach(4);
  trapdoor.write(0);

}

void loop() {   
  //height();
  //gatedetector();  
  //colorline();
  //dashedline();
  // Example condition
  //Serial.println("trying");
  //grab();
  //Serial.println("ohase2");
  //grelease();
  //height();
  //loopfollowfull();
  //mdrive(0,0);
  //delay(1000);
  //heightnew();
  //Height2=Height2-14;
  //go(Height2);
  //back(15);
  //turnleftfull();
  //loopfollow();
  //mdrive(0,0);
  //delay(1000);
  //back(20);
  //mdrive(0,0);
  //delay(100000);
  //colorline();
  //dashedline();
  //loopfollowfull();
  //mdrive(0,0);
  //back(20);
  //go(Height2);
  //mdrive(0,0);
  //delay(100000);
    // Other loop operations...
  //processLineFollowing();
  //gatetimer();
  //if(r==0){
    //four();
    //r++;
  //}
  //mdrive(0,0);
  //sensor.setColorMode(LINE_BLACK);
  //tasksixascending1();
  /*for(int i = 22 ; i<30 ;i++){
  Serial.print(digitalRead(i));
  }
  Serial.println();
  mdrive(0,0);*/
  //delay(10000);
  //forward();
  //gatedetector();
  //reverse();
  //delay(5000);
  //reverse();
  //delay(500000);
  /*processLineFollowing();
  if(checkrightsensors()){
    mdrive(0,0);
    forward();
    turnright();
    delay(5000);
  }*/
  //turnrightfull();
  //delay(5000);
  //turnleftfull();
  //reverse();
  //mdrive(0,0);
  //delay(5000);
  //correction();

  //Serial.println(digitalRead(40));
  //delay(1000);

  if(count==0 && digitalRead(40) == LOW){
    while(count < 3){
    // set target position
      int target[NMOTORS];
      target[0] = 6000;
      target[1] = 6000;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      
      if(k==0){
        setMotor(dir,pwr-2.5,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    int b = analogRead(A3);
    start_pos  = pos[0];
    if(digitalRead(22) == 0 and digitalRead(29)==0 ){
      start_pos = pos[0];
      //Serial.println(start_pos);
      while(digitalRead(22) == 0 and digitalRead(29)==0 ){

      }
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        for(int k = 0; k < NMOTORS; k++){
          pos[k] = posi[k];
        }
      } 
      end_pos = pos[0];
      //Serial.println(end_pos);
      difference = end_pos - start_pos;
      //Serial.println(difference);
      
    }

    if(q==0){
      q++;
      difference=0;
    }
    else{
          if (difference > 200 ){
            barcode.add(1);
            difference = 0;
            count = 0;

          }
          if(difference != 0 && difference <200 && difference >80){
            barcode.add(0);
            difference = 0;
            count++;
          }
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );
  }
  // Barcode Calculate
    if(p == 0){
    
      for (int k = barcode.size() - 4; k >= 0; k--) {
        if (p < 2) {
          number += (1 << p) * barcode.get(k);  // Use bit shifting instead of pow
          p++;
        } 
        else {
          number += (1 << p) * barcode.get(k);  // Use bit shifting here as well
          p++;
        }
      }
    }
    count++;
    
  
    if (count == 4){
      Serial.print("number ");
      Serial.println(number);
      delay(1000);
      setMotor(1, 0 , 6, 7 );
      setMotor(1, 0 , 8 , 9 );
      count++;
      loopfollowfull();
  }
  
  //reverse();
  //forward();
  //turnleft();

  while(r<1){
    processLineFollowing();
    if (checkrightsensors()){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      r++;
    }
  }
  
  if(r==1){
    num = number % 5; 
    if(num==0){  
      zero();
    }
    if(num==1){  
      one();
    }
    if(num==2){  
      two();
    }
    if(num==3){  
      three();
    }
    if(num==4){  
      four();
    }
    r++;
    forward();
    mdrive(0,0);
    delay(1000);
  }
   

  //if(r==2){
    //colorline();
    //dashedline();
    //r++;
  //}
    if(r==2 ){
      colorline();
      r++;
    }
  }  
  else{
    r=3;
  }
  if(r==3 && digitalRead(38)==LOW){
    task3=1;
    dashedline();
    gatetimer();
    delay(1500);
    go(40);
    
    mdrive(0,0);
    r++;
  }
  else{
    r=4;
  }

  sensor.setColorMode(LINE_BLACK);
  if(r==4 && digitalRead(37)==LOW){
    if(order){
      tasksixascending1();
      tasksixascending2();
      tasksixascending3();
      chamber1();
      r++;
      mdrive(0,0);    
    }
    else{
      tasksixascending11();
      tasksixascending22();
      tasksixascending33();
      r++;
      chamber1();
      mdrive(0,0);     
    }
  }
  else{
    r=5;
  }
  sensor.setColorMode(LINE_BLACK);
  if(r==5 && digitalRead(34) == 0){
    while(dul < 3){
    processLineFollowing();
    if((checkallsensorsB() or checkrightsensorsB() or checkallsensorsB()) && dul ==0){
      forward();
      loopfollowfull();
      dul++;
    }
    if((checkallsensorsB() or checkrightsensorsB() or checkallsensorsB()) && dul ==1){
      mdrive(0,0);
      delay(1000);     
      forward();

      turnright();
      delay(1000);
      loopfollowB();
      forwardminisix();
      mdrive(0,0);
      delay(1000);     
      forwardminisix();
      mdrive(0,0);
      delay(2000);
      reversesix();
      mdrive(0,0);
      delay(1000);
      reversesix();
      mdrive(0,0);
      delay(1000);
      turnrightfull();
      loopfollowfull();
      mdrive(0,0);
      dul++;
      }
    if((checkallsensorsB() or checkrightsensorsB() or checkallsensorsB()) && dul ==2){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      loopfollowfull();
      mdrive(0,0);
      delay(1000);
      dul++;
    }
    }
    r++;
  
  }
  else{
    r=6;
  }
  if(r==6 && digitalRead(32) == 0){
    chamber2();
    mdrive(0,0);
    while ((digitalRead(29)==1) or (digitalRead(28)==1) or (digitalRead(27)==1) or (digitalRead(26)==1) or
    (digitalRead(25)==1) or (digitalRead(24)==1) or (digitalRead(23)==1) or (digitalRead(22)==1)){
      turnrighthalf();
      mdrive(80,80);
      delay(4500);
      turnlefthalf();
      mdrive(80,80);

      }
    go(10);
    openDoor;
    r++;
  }
  
  

}


void setMotor(int dir, int pwmVal, int in1, int in2){
  
  if(dir == 1){
    analogWrite(in1,pwmVal);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    analogWrite(in2,pwmVal);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}
void turnright(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 620+200;
    target[1] = -620-200;
    while((posi[0]<target[0]-70-200)){
    // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void turnleft(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -580-200;
    target[1] = 580+200;
    while((posi[1]<target[1]-200)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void forward(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 310+200;
    target[1] = 310+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }

}
int mdrive(int m1,int m2){
  if(m1>0){
    //m forward
    if (m1>255){
      m1=255;
    }
    digitalWrite(M1,HIGH);
    analogWrite(M1pwm,255-m1);
  }
  else{
    if (m1<-255){
      m1=-255;
    }
    digitalWrite(M1,LOW);
    analogWrite(M1pwm,m1*-1);
  }
  if(m2>0){
    //m forward
    if (m2>255){
      m2=255;
    }
    digitalWrite(M2,HIGH);
    analogWrite(M2pwm,255-m2);
  }
  else{
    if (m2<-255){
      m2=-255;
    }
    digitalWrite(M2,LOW);
    analogWrite(M2pwm,m2*-1);
  }
  

}
void processLineFollowing() {
  
  int err = sensor.readSensor(); //calculating the error // 

    for (int i = 0; i < 8; i++) {
    Serial.print(sensor.values[i]);
    Serial.print('\t');
    }
  //Serial.println(err); //printing the error value//
  
    int m1 = 80;
    int m2 = 80;
  
    int diff = err * kP + (err - last_value) * kD; // we will add this to one motor and subtract from other //
  
    last_value = err;
  //Serial.println(diff);    
  mdrive(m1 + diff, m2 - diff);
  Serial.println(diff); //printing the error value//

  
}
int checkallsensors(){
  bool allsensors;
  if(((digitalRead(29)==0)&&(digitalRead(22)==0)))
  {
    allsensors=true;
  }
  else{
    allsensors=false;
  }
  return allsensors;
}
int checkrightsensors(){
  bool rightsensors;
  //((digitalRead(43)==0)&&(digitalRead(42)==0)&&(digitalRead(41)==0)&&(digitalRead(40)==0)&&(digitalRead(36)==1))||
  if((digitalRead(29)==0)&&(digitalRead(28)==0)&&(digitalRead(27)==0)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(22)==1))
    {
      rightsensors=true; 
    }
  else{
    rightsensors=false;
  }
  return rightsensors;
}

int checkleftsensors(){
  bool leftsensors;
  //((digitalRead(43)==1)&&(digitalRead(39)==0)&&(digitalRead(38)==0)&&(digitalRead(37)==0)&&(digitalRead(36)==0))||
  if((digitalRead(29)==1)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0))
  {
    leftsensors=true;
  }
  else{
    leftsensors=false;
  }
  return leftsensors;
}

void forwardmini(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 50+200;
    target[1] = 50+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void forwardtask3(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 200+200;
    target[1] = 200+30+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );
}
void forwardtask2(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 200+200;
    target[1] = 200+200;
    while(posi[0]<target[0]-30-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );
}
void reversemini(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -250-200;
    target[1] = -250-200;
    while(posi[0]>target[0]+30+200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr-2.5,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void reverse(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -1600-200;
    target[1] = -1600-200;
    while(posi[0]>target[0]+40+100){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }

    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void turnleftfull(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -1150-200;
    target[1] = 1150+200;
    while((posi[1]<target[1]-75-200)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );



}
void turnrightfull(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 1150+200;
    target[1] = -1150-200;
    while((posi[0]<target[0]-50-200)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 ); 

}
void zero(){
  while(counttask2<20){
    processLineFollowing();
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 0){
      mdrive(0,0);
      digitalWrite(13,HIGH);
      forwardtask2();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 1){
      mdrive(0,0);
      delay(1000);
      digitalWrite(13,LOW);;
      delay(1000);
      reverse();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 2){
      forward();
      turnright();
      loopfollow(); 
      counttask2++; 
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 3){
      mdrive(0,0);
      delay(1000);
      forward();
      turnleft();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
      mdrive(0,0);
      delay(2000);
      forward();
      turnright();
      mdrive(0,0);
      gatedetector();
      delay(1000);
      turnleftfull();
      loopfollow();

      counttask2++;
    }
    if (gate==true){
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(2000);
        digitalWrite(13,LOW);
        reversemini();
        turnleftfull();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 13){
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 14){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 15){
        mdrive(0,0);
        reversemini();
        turnrightfull();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 16){
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 17){
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }        
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 18){
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 19){
        digitalWrite(13,HIGH);
        forward();
        forward();
        forward();
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        counttask2++;
      }
    }
    else{
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(2000);
        digitalWrite(13,LOW);
        reversemini();
        turnleftfull();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        forward();
        mdrive(0,0);
        delay(1000);
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        mdrive(0,0);
        digitalWrite(13,HIGH);
        go(35);
        mdrive(0,0);
        digitalWrite(13,LOW);
        mdrive(0,0);
        counttask2+=10;
      }
    }
  }
}
void one(){
while(counttask2<13){
    processLineFollowing();
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 0){
      mdrive(0,0);
      forwardtask2();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 1){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 2){
      mdrive(0,0);
      delay(1000);
      gatedetector();
      counttask2++;

    }
    if (gate){
      if (counttask2 == 3){
          turnleftfull();
          loopfollowfull();
          counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow(); 
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnrightfull();
        loopfollowfull();        
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        turnrightfull();
        mdrive(0,0);
        counttask2++;
      }
    }
    else{
      if(counttask2 == 3){
        mdrive(0,0);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        mdrive(0,0);
        digitalWrite(13,LOW);
        turnleftfull();
        mdrive(0,0);
        counttask2+=20;
      }
      
    }
  }  
}
void two(){
while(counttask2<13){
    processLineFollowing();
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 0){
      mdrive(0,0);
      forwardtask2();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 1){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 2){
      mdrive(0,0);
      delay(1000);
      gatedetector();
      counttask2++;

    }
    if (gate){
      if (counttask2 == 3){
          turnrightfull();
          loopfollowfull();
          counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        digitalWrite(13,HIGH);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnrightfull();  
        loopfollowfull();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        delay(1000);
        digitalWrite(13,LOW);
        turnrightfull();
        mdrive(0,0);
        counttask2++;
      }
    }
    else{
      if(counttask2 == 3){
        mdrive(0,0);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(2000);
        digitalWrite(13,HIGH);
        reverse();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnleftfull();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);        
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        mdrive(0,0);
        digitalWrite(13,HIGH);
        reverse();
        digitalWrite(13,LOW);
        mdrive(0,0);
        delay(1000);
        turnleftfull();
        mdrive(0,0);
        counttask2+=10;
      }
    }
  }  

}
void three(){
while(counttask2<15){
    processLineFollowing();
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 0){
      mdrive(0,0);
      forwardtask2();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 1){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 2){
      mdrive(0,0);
      delay(1000);
      gatedetector();
      counttask2++;

    }
    if (gate){
      if (counttask2 == 3){
          forward();
          loopfollow();
          counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        digitalWrite(13,LOW);
        turnleftfull();       
        counttask2+=10;
      }

    }
    else{
      if(counttask2 == 3){
        mdrive(0,0);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);        
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnrightfull();
        loopfollowfull();
        mdrive(0,0);
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        mdrive(0,0);
        delay(1000);        
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);        
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 13){
        mdrive(0,0);
        delay(1000);        
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 14){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        digitalWrite(13,LOW);
        turnleftfull();
        loopfollowfull();
        mdrive(0,0);
        counttask2++;
      }
    }
  }  
 
}
void four(){
while(counttask2<21){
    processLineFollowing();
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 0){
      mdrive(0,0);
      forwardtask2();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 1){
      mdrive(0,0);
      delay(1000);
      forward();
      turnright();
      loopfollow();
      counttask2++;
    }
    if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 2){
      mdrive(0,0);
      delay(1000);
      gatedetector();
      counttask2++;

    }
    if (gate){
      if (counttask2 == 3){
          forward();
          turnright();
          loopfollow();
          counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnrightfull(); 
        loopfollowfull();     
        counttask2++;
      }
 
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        delay(1000);
        digitalWrite(13,LOW);
        turnrightfull();
        loopfollowfull();
        counttask2+=10;
      }
    }
    else{
      if (counttask2 == 3){
          forward();
          turnright();
          loopfollow();
          counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 4){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 5){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 6){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 7){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 8){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnrightfull();
        loopfollowfull();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 9){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 10){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();      
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 11){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 12){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();    
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 13){
        mdrive(0,0);
        delay(1000);
        forward();
        turnright();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 14){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 15){
        mdrive(0,0);
        delay(1000);
        forward();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 16){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        reversemini();
        turnleftfull();
        loopfollowfull();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 17){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 18){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 19){
        mdrive(0,0);
        delay(1000);
        forward();
        turnleft();
        loopfollow();
        counttask2++;
      }
      if((checkrightsensors() or checkleftsensors() or checkallsensors()) and counttask2 == 20){
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,HIGH);
        reverse();
        mdrive(0,0);
        delay(1000);
        digitalWrite(13,LOW);
        turnrightfull();
        mdrive(0,0);
        counttask2+=10;
      }
    }
  }    
}
void correction(){
  
    for(int i=0;i<15000;i++){
      mdrive(-100,100);
      if((digitalRead(39)==0)&&(digitalRead(40)==0)){
        mdrive(0,0);
        return;
      }
      }
    for(int i=0;i<30000;i++){
      mdrive(100,-100);
      if((digitalRead(39)==0)&&(digitalRead(40)==0)){
        mdrive(0,0);
        return;
      }
      }
    for(int i=0;i<15000;i++){
      mdrive(-100,100);
      if((digitalRead(39)==0)&&(digitalRead(40)==0)){
        mdrive(0,0);
        return;
      }
      }    
}

void gatedetector(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin and calculate the distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound = 0.034 cm/us

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(1000);
  Height3=distance;
  if (distance < 35 && distance != 0){
    Serial.print("Done");
    gate = true;
    order = false;
  }
  else{
    gate = false;
    order = true;
  }
}
void loopfollow(){
      for(int j=0;j<50;j++){
      processLineFollowing();
    }
}
void loopfollowfull(){
      for(int j=0;j<20;j++){
      processLineFollowing();
    }
}

int checkallsensorsB(){
  bool allsensors;
  if(((digitalRead(29)==1)&&(digitalRead(22)==1)))
  {
    allsensors=true;
  }
  else{
    allsensors=false;
  }
  return allsensors;
}

int checkrightsensorsB(){
  bool rightsensors;
  //((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(22)==0))||
  if((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(25)==1)&&(digitalRead(22)==0))
    {
      rightsensors=true; 
    }
  else{
    rightsensors=false;
  }
  return rightsensors;
}

int checkleftsensorsB(){
  bool leftsensors;
  //((digitalRead(43)==1)&&(digitalRead(39)==0)&&(digitalRead(38)==0)&&(digitalRead(37)==0)&&(digitalRead(36)==0))||
  if((digitalRead(29)==1)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0))
  {
    leftsensors=true;
  }
  else{
    leftsensors=false;
  }
  return leftsensors;
}

void tasksixascending1(){
  while(countsix < 6){
    processLineFollowing();
    if((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix == 0){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(2000);
      height();
      delay(1000);
      if(Height == 5){
        digitalWrite(13,HIGH);
      }
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      //delay(10000);
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 5){
      mdrive(0,0);
      delay(5000);
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      reversesix();
      turnleftfull();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 5){
      forward();
      turnright();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 5){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      delay(1000);
      grab();
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 10){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 10){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 10){
      forward();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 15){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 15){
      mdrive(0,0);
      delay(1000);
      forward();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 15){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==4 and Height == 15){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==5 and Height == 15){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
  }
}

void tasksixascending2(){
  while(countsix2 < 5){
    processLineFollowing();
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 5){
      forward();
      turnright();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 5){
      forward();
      turnleft();
      loopfollow();
      forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 5){
      forward();
      turnright();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==3 and Height == 5){
      forward();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==4 and Height == 5){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix2+=10;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 10){
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 10){
      forward();
      turnright();
      loopfollow();
      countsix2++;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 10){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      height();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      delay(1000);
      countsix2+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 15){
      forward();
      turnleft();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 15){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      delay(1000);
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 15){
      forward();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      reversesix();      
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix2+=10;
    }
  }
}


void tasksixascending3(){
  while(countsix3 < 8){
    processLineFollowing();
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 5){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 5){
      forward();
      loopfollow();
      countsix3++;
    }

    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==2 and Height == 5){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==3 and Height == 5){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==4 and Height == 5){
      forward();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==5 and Height == 5){
      forward();
      mdrive(0,0);
      countsix3+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 10){
      forward();
      turnright();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 10){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==2 and Height == 10){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==3 and Height == 10){
      forward();
      mdrive(0,0);
      delay(1000);
      countsix3+=10;
    }

    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 15){
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 15){
      forward();
      turnright();
      mdrive(0,0);
      countsix3+=10;
    }
  }
}

long measureDistance(int trigPinp, int echoPinp) {
  digitalWrite(trigPinp, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinp, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinp, LOW);

  long duration = pulseIn(echoPinp, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void height() {
  // Ultrasonic Sensor 1
  digitalWrite(30, HIGH);
  delay(1000);
  long distance1 = measureDistance(trigPin1, echoPin1);
  Serial.print("Ultrasonic Sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  if (distance1 < 10 && distance1 != 0) {
    height1 = true;
  } else {
    height1 = false;
  }
  digitalWrite(30, LOW);
  delay(1000);

  // Ultrasonic Sensor 2
  digitalWrite(33, HIGH);
  delay(1000);
  long distance2 = measureDistance(trigPin2, echoPin2);
  Serial.print("Ultrasonic Sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");
  if (distance2 < 10 && distance2 != 0) {
    height2 = true;
  } else {
    height2 = false;
  }
  digitalWrite(33, LOW);

  // Ultrasonic Sensor 3 (Replacing ToF sensor)
  digitalWrite(42, HIGH); // This is pin 43, you can turn it on or off as needed
  delay(1000); // Add a small delay before measuring distance
  long distance3 = measureDistance(trigPin3, echoPin3);
  Serial.print("Ultrasonic Sensor 3: ");
  Serial.print(distance3);
  Serial.println(" cm");
  digitalWrite(42, HIGH); // This is pin 43, you can turn it on or off as needed
  delay(1000);
  if (distance3 > 0 && distance3 < 10) { // Check if the reading is within a reasonable range
    height3 = true;
  } else {
    height3 = false;
  }

  // Print height determination logic
  if (height1 && height2 && height3) {
    Height = 15;
    Serial.println("Height: 15");
  } else if (!height1 && height2 && height3) {
    Height = 10;
    Serial.println("Height: 10");
  } else if (!height1 && !height2 && height3) {
    Height = 5;
    Serial.println("Height: 5");
    digitalWrite(13,HIGH);
  }
}

void grab(){
  int x=30;
  for (int x=30;x<200;x++){
    myservo.write(x);
    delay(10);
  }
}

void grelease(){
  int x=200;
  for (int x=200;x>30;x--){
    myservo.write(x);
    delay(10);
  }
}

void openDoor(){
  trapdoor.write(0);
}

void closeDoor(){
  trapdoor.write(70);
}

void gatetimer(){
  digitalWrite(42,HIGH);
  delay(1000);
  dstart = millis();
  while(ddifference < 4500){
    delay(100);
    while(measureDistance(trigPin3, echoPin3) < 10 && measureDistance(trigPin3, echoPin3) != 0){
    delay(100);
    }
    dend = millis();
    ddifference=dend - dstart;
    dstart = millis();
  }
  digitalWrite(42,LOW);
  Serial.println("Done");
}



void reversesix(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -250-200;
    target[1] = -250-200;
    while(posi[0]>target[0]+40+100){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }

    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}

void forwardsix(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 280+200;
    target[1] = 280+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 ); 
}
void forwardminisix(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 225+200;
    target[1] = 225+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 ); 
}

  /*processLineFollowing();
  if(checkrightsensors() || checkleftsensors()){
    forwardmini();
    mdrive(0,0);
    delay(2000);

    if(checkallsensors()){
        Serial.println("Done");
        //for(i=53;i>45;i--){
          //Serial.print(digitalRead(i));
          //}
        forward();

        for(int j=0;j<15;j++){
          processLineFollowing();
        }
    } 
    if(checkrightsensors()){
        mdrive(0,0);
        //Serial.println();
        //for(i=53;i>45;i--){
          //Serial.print(digitalRead(i)); 
        //}
        mdrive(0,0);
        forwardtask3();
        mdrive(0,0);
        turnright();
        for(int j=0;j<15;j++){
          processLineFollowing();
      }
    }
    
  

    if(checkleftsensors()){
        //Serial.println();
        //for(i=53;i>45;i--){
          //Serial.print(digitalRead(i));
          //}
        mdrive(0,0);
        forwardtask3();
        mdrive(0,0);
        turnleft();
        for(int j=0;j<15;j++){
          processLineFollowing();
        }
    }

  }*/

void tasksixascending11(){
  while(countsix < 6){
    processLineFollowing();
    if((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix == 0){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 15){
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 15){
      forward();
      turnright();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 15){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      delay(1000);
      grab();
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 10){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 10){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 10){
      forward();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==1 and Height == 5){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==2 and Height == 5){
      mdrive(0,0);
      delay(1000);
      forward();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==3 and Height == 5){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==4 and Height == 5){
      forward();
      turnleft();
      loopfollow();
      countsix++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix==5 and Height == 5){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix+=10;
    }
  }
}
void tasksixascending22(){
  while(countsix2 < 5){
    processLineFollowing();
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 15){
      forward();
      turnright();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 15){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 15){
      forward();
      turnright();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==3 and Height == 15){
      forward();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==4 and Height == 15){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      delay(1000);
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollow();
      mdrive(0,0);
      countsix2+=10;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 10){
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 10){
      forward();
      turnright();
      loopfollow();
      countsix2++;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 10){
      forward();
      turnleft();
      loopfollowB();
      mdrive(0,0);
      //forwardsix();
      height();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      delay(1000);
      countsix2+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==0 and Height == 5){
      forward();
      turnleft();
      loopfollow();
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==1 and Height == 5){
      forward();
      turnright();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      delay(1000);
      countsix2++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix2==2 and Height == 5){
      forward();
      loopfollowB();
      //forwardsix();
      mdrive(0,0);
      delay(1000);
      height();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      reversesix();      
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix2+=10;
    }
  }
}
void tasksixascending33(){
  while(countsix3 < 8){
    processLineFollowing();
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 15){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 15){
      forward();
      loopfollow();
      countsix3++;
    }

    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==2 and Height == 15){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;      
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==3 and Height == 15){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==4 and Height == 15){
      forward();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==5 and Height == 15){
      forward();
      mdrive(0,0);
      countsix3+=10;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 10){
      forward();
      turnright();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 10){
      forward();
      turnleft();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==2 and Height == 10){
      forward();
      turnright();
      loopfollow();
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==3 and Height == 10){
      forward();
      mdrive(0,0);
      delay(1000);
      countsix3+=10;
    }

    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==0 and Height == 5){
      forward();
      loopfollowB();
      //forwardsix();
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grelease();
      delay(1000);
      reversesix();
      turnleftfull();
      loopfollowfull();
      mdrive(0,0);
      countsix3++;
    }
    if ((checkrightsensorsB() or checkleftsensorsB() or checkallsensorsB()) and countsix3==1 and Height == 5){
      forward();
      turnright();
      mdrive(0,0);
      countsix3+=10;
    }
  }
}
void forwardminicolorline(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 90;
    target[1] = 90;
    while(posi[0]<target[0]-20){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
int checkleftsensorscolorline(){
  bool leftsensors;
  //((digitalRead(43)==1)&&(digitalRead(39)==0)&&(digitalRead(38)==0)&&(digitalRead(37)==0)&&(digitalRead(36)==0))||
  if(((digitalRead(29)==1)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0))||
      ((digitalRead(29)==1)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0)))
  {
    leftsensors=true;
  }
  else{
    leftsensors=false;
  }
  return leftsensors;
}
void forwardminimini(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 50;
    target[1] = 50;
    while(posi[0]<target[0]-20){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void forwardcolorline(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 100;
    target[1] = 100;
    while(posi[0]<target[0]-20){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );


}
int checkrightsensorscolorline(){
  bool rightsensors;
  //((digitalRead(43)==0)&&(digitalRead(42)==0)&&(digitalRead(41)==0)&&(digitalRead(40)==0)&&(digitalRead(36)==1))||
  if(((digitalRead(29)==0)&&(digitalRead(28)==0)&&(digitalRead(27)==0)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(22)==1))||
    ((digitalRead(29)==0)&&(digitalRead(28)==0)&&(digitalRead(27)==0)&&(digitalRead(26)==0)&&(digitalRead(22)==1)))

    {
      rightsensors=true; 
    }
  else{
    rightsensors=false;
  }
  return rightsensors;
}

void colorline(){
  while (task3==0){
    while (starttask==0){
      processLineFollowing();
      if ((digitalRead(22)==1) and (digitalRead(29)==1)){
        starttask=1;
      }
    }
    processLineFollowing();

    //5 or 4 IR are white from the right side
    if(checkrightsensorscolorline() and task3==0){
      forwardminicolorline();

      if(checkrightsensorscolorline() and task3==0){
        forwardminicolorline();
        forwardminicolorline();

        if(checkallblack() and task3==0){
          mdrive(0,0);
          delay(200);
          forwardcolorline();
          mdrive(0,0);
          delay(200);
          turnrightcolorline();
          for(int j=0;j<20;j++){
            processLineFollowing();
          }
        }
        

        if((checkall())||(checkleftsensors())||(checkrightsensors())){
          while (colorlinecorrection==0){
            processLineFollowing();
            if((digitalRead(29)==1)&&(digitalRead(22)==0)){
              turnleftmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==0)&&(digitalRead(22)==1)){
              turnrightmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==1)&&(digitalRead(22)==1)){
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
          }
        }
      }     
    }

    //5 or 4 LEDs in the IR array are white from the left 
    if(checkleftsensorscolorline() and task3==0){
      forwardminicolorline();
      if(checkleftsensorscolorline() and task3==0 ){
        forwardminicolorline();
        forwardminicolorline();
        
        if(checkallblack() and task3==0){
          mdrive(0,0);
          delay(200);
          forwardcolorline();
          mdrive(0,0);
          delay(200);
          turnleftcolorline();
          for(int j=0;j<20;j++){
            processLineFollowing();
          }
        }
        if((checkall())||(checkleftsensors())||(checkrightsensors())){
          while (colorlinecorrection==0){
            processLineFollowing();
            if((digitalRead(29)==1)&&(digitalRead(22)==0)){
              turnleftmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==0)&&(digitalRead(22)==1)){
              turnrightmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==1)&&(digitalRead(22)==1)){
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
          }
        }



        
    }
    }
    //all IR are in a white background
    if(checkall()){

      int allwhite=0;
      for(int i=0;i<20;i++){
        processLineFollowing();
      }

      if (checkall()){
          mdrive(0,0);
          task3=1;
          task4=0;
          while (colorlinecorrection==0){
            processLineFollowing();
            if((digitalRead(29)==1)&&(digitalRead(22)==0)){
              //turnleftmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==0)&&(digitalRead(22)==1)){
              //turnrightmini();
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
            if((digitalRead(29)==1)&&(digitalRead(22)==1)){
              mdrive(0,0);
              task3=1;
              colorlinecorrection=1;

            }
          }
          
      }
      

      }
      

      
    }
}
void dashedline(){
  while (task3==1 and task4==0){
    processLineFollowing();
  
  if ((digitalRead(22)==1) and (digitalRead(23)==1) and (digitalRead(24)==1) and (digitalRead(25)==1) and (digitalRead(26)==1) and (digitalRead(27)==1) and (digitalRead(28)==1) and (digitalRead(29)==1) and stopcount==0 and task4==0 and task3==1){
    task4linecount++;

  }
  if ((digitalRead(22)==0) and (digitalRead(23)==0) and (digitalRead(24)==0) and (digitalRead(25)==0) and (digitalRead(26)==0) and (digitalRead(27)==0) and (digitalRead(28)==0) and (digitalRead(29)==0) and task4==0 and task3==1 and task4linecount>1){
    digitalWrite(13,HIGH);
    for(int i=0;i<10;i++){
      processLineFollowing();
    }
    mdrive(0,0);
    task4=1;
  }
  } 
}

void turnleftcolorline(){
   mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -620-200;
    target[1] = 620+200;
    while((posi[1]<target[1]-300)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}                                                                                                                                    void turnrightcolorline(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 620+200;
    target[1] = -620-200;
    while((posi[0]<target[0]-70-250)){
    // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
int checkall(){
  bool allsensors;
  if(((digitalRead(29)==0)&&(digitalRead(28)==0)&&(digitalRead(27)==0)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&
  (digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0)))
  {
    allsensors=true;
  }
  else{
    allsensors=false;
  }
  return allsensors;
}
int checkallblack(){
  bool allsensors;
  if(((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(25)==1)&&(digitalRead(24)==1)&&(digitalRead(23)==1)&&(digitalRead(22)==1)))
  {
    allsensors=true;
  }
  else{
    allsensors=false;
  }
  return allsensors;
}
void turnrightmini(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 200;
    target[1] = -200;
    while((posi[0]<target[0]-20)){
    // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }

}
void turnleftmini(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -200;
    target[1] = 200;
    while((posi[1]<target[1]-20)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }

}
void Task8A(){
  while(countP < 4){
    processLineFollowingB();
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countP==0){
      forward();
      forward();
      turnleft();
      forward();
      loopfollowfull();
      mdrive(0,0);
      countP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countP==1){
      mdrive(0,0);
      delay(10000);
      forward();
      turnleft();
      loopfollowfull();
      countP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countP==2){
      forwardminisix();
      mdrive(0,0);
      delay(1000);
      grab();
      delay(1000);
      turnleftfull();
      loopfollowfull();
      countP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countP==3){
      forward();
      turnleft();
      loopfollow();
      mdrive(0,0);
      delay(1000);
      countP++;
    }
  }
}

void Task8B(){
  while(countPP < 5){
    processLineFollowingB();
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB()) && countPP==0){
      forward();
      loopfollow();
      mdrive(0,0);
      heightnew();
      Height2=Height2-14;
      go(Height2);
      grelease();
      back(15);
      turnleftfull();
      mdrive(0,0);
      delay(1000);
      reverseseven();
      loopfollowfull();
      countPP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countPP==1){
      forward();
      turnright();
      loopfollow();
      countPP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countPP==2){
      forward();
      countPP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countPP==3){
      forward();
      countPP++;
    }
    if((checkallsensorsB() or checkrightsensorsB1() or checkleftsensorsB1()) && countPP==4){
      forward();
      mdrive(0,0);
      countPP++;
      delay(10000);
    }
  }
}


void go(int DISTANCE){
    int DISTANCEP = 41*DISTANCE;
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = DISTANCEP+200;
    target[1] = DISTANCEP+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );   

}

void heightnew() {
  // Ultrasonic Sensor 2
  digitalWrite(33, HIGH);
  delay(1000);
  long distancenew = measureDistance(trigPin2, echoPin2);
  Serial.print("Ultrasonic Sensor 2: ");
  Serial.print(distancenew);
  Serial.println(" cm");
  Height2 = distancenew;
  delay(1000);
  digitalWrite(33,LOW);
}

void back(int DISTANCEB){
    int DISTANCEBP = 41*DISTANCEB +40;
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -DISTANCEBP-200;
    target[1] = -DISTANCEBP-200;
    while(posi[0]>target[0]+40+100){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }

    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );  
}



void reverseseven(){
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -900-200;
    target[1] = -900-200;
    while(posi[0]>target[0]+40+250){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }

    }

    for(int k = 0; k < NMOTORS; k++){
      //Serial.print(target[k]);
      //Serial.print(" ");
      //Serial.print(pos[k]);
      //Serial.print(" ");
    }
      //Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );
}

void loopfollowB(){
      for(int j=0;j<80;j++){
      processLineFollowing();
    }
    mdrive(0,0);
}
int checkrightsensorsB1(){
  bool rightsensors;
  if(((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(22)==0))||
  ((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(25)==1)&&(digitalRead(22)==0)))
    {
      rightsensors=true; 
    }
  else{
    rightsensors=false;
  }
  return rightsensors;
}

int checkleftsensorsB1(){
  bool leftsensors;
  if(((digitalRead(29)==1)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0))||
  ((digitalRead(29)==1)&&(digitalRead(26)==0)&&(digitalRead(25)==0)&&(digitalRead(24)==0)&&(digitalRead(23)==0)&&(digitalRead(22)==0)))
  {
    leftsensors=true;
  }
  else{
    leftsensors=false;
  }
  return leftsensors;
}

void processLineFollowingB() {
  
  int err = sensor.readSensor(); //calculating the error // 

    for (int i = 0; i < 8; i++) {
    Serial.print(sensor.values[i]);
    Serial.print('\t');
    }
  //Serial.println(err); //printing the error value//
  
    int m1 = 70;
    int m2 = 70;
  
    int diff = err * kP + (err - last_value) * kD; // we will add this to one motor and subtract from other //
  
    last_value = err;
  //Serial.println(diff);    
  mdrive(m1 + diff, m2 - diff);
  Serial.println(diff); //printing the error value//

  
}
void forwardtask7_new(){
  int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 100+200;
    target[1] = 100+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }

}
void forwardtask7(){
  int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 500+200;
    target[1] = 500+200;
    while(posi[0]<target[0]-20-200){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }

}
int checkleftblack(){
  bool leftsensors;
  //((digitalRead(43)==1)&&(digitalRead(39)==0)&&(digitalRead(38)==0)&&(digitalRead(37)==0)&&(digitalRead(36)==0))||
  if(((digitalRead(29)==0)&&(digitalRead(26)==1)&&(digitalRead(25)==1)&&(digitalRead(24)==1)&&(digitalRead(23)==1)&&(digitalRead(22)==1))||
      ((digitalRead(29)==0)&&(digitalRead(25)==1)&&(digitalRead(24)==1)&&(digitalRead(23)==1)&&(digitalRead(22)==1)))
  {
    leftsensors=true;
  }
  else{
    leftsensors=false;
  }
  return leftsensors;
}
int checkrightblack(){
   bool rightsensors;
  //((digitalRead(43)==0)&&(digitalRead(42)==0)&&(digitalRead(41)==0)&&(digitalRead(40)==0)&&(digitalRead(36)==1))||
  if(((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(25)==1)&&(digitalRead(22)==0))||
    ((digitalRead(29)==1)&&(digitalRead(28)==1)&&(digitalRead(27)==1)&&(digitalRead(26)==1)&&(digitalRead(22)==0)))

    {
      rightsensors=true; 
    }
  else{
    rightsensors=false;
  }
  return rightsensors;
}
void chamber1(){
  while(task7_1==0){
    while(allblack==0 and linefollowingcount==0){
    processLineFollowing();
    if(checkallblack()){
      mdrive(0,0);
      delay(500);
      linefollowingcount++;//1
    }
    }
    if (allblack==0 and linefollowingcount==1){
      forwardtask7();
      turnleft();
      mdrive(0,0);
      allblack++;//1
    }
    while(allblack==1 and linefollowingcount==1){
      if(black==0){
        forwardtask7_new();
        black++;
      }
      processLineFollowing();
      if(checkleftblack()){
        mdrive(0,0);
        linefollowingcount++; //2
    }
    }
    if (allblack==1 and linefollowingcount==2){
      forward();
      turnleft();
      mdrive(0,0);
      allblack++; //2
      
    }
    while(allblack==2 and linefollowingcount==2){
     processLineFollowing();
     if(checkallblack()){
        mdrive(0,0);
        linefollowingcount++; //3
        delay(1000);
        grab();
        delay(1000);
        
        //grab();
      }
    }
    if (allblack==2 and linefollowingcount==3){
      delay(1000);
      turnleftfull();
      mdrive(0,0);
      allblack++;//3
    }
    while(allblack==3 and linefollowingcount==3){
      processLineFollowing();
      if(checkallblack()){
        mdrive(0,0);
        linefollowingcount++; //4
      }
    }
    if (allblack==3 and linefollowingcount==4){
      forward();
      turnleft();
      forward();
      mdrive(0,0);
      allblack++;//4
      task7_1=1;
    }

  }
}

void chamber2(){
  while(task7_2==0){
      while(allblack2==0 and linefollowingcount2==0){
      processLineFollowing();
      if(checkleftblack()){
      turnright();
      delay(1000);
      mdrive(0,0);
      linefollowingcount2++; //1
    }
    }
    if (allblack2==0 and linefollowingcount2==1){
      gatedetector();
      
      Height3=Height3-14;

      turnright();
      delay(4000);
      turnright();
      delay(1000);
      turnright();
      
      delay(1000);
      go(Height3);
      grelease();
      back(15);
      turnleftfull();
      mdrive(0,0);
      delay(1000);
      reverseseven();
      loopfollowfull();
      allblack2++;//1
    }
    while(allblack2==1 and linefollowingcount2==1){
      processLineFollowing();
      if(checkrightblack()){
        mdrive(0,0);
        forward();
        turnright();
        mdrive(0,0);
        linefollowingcount2++; //2
      
      }
    }
    while(allblack2==1 and linefollowingcount2==2){
      processLineFollowing();
      if(checkallblack() and blackcount==0 and whitecount==0){
        blackcount++;
      }
      if(checkall() and whitecount==0 and blackcount==1){
        whitecount++;
      }
      if(checkallblack() and blackcount==1 and whitecount==1){
        blackcount++;
      }
      if(checkall() and whitecount==1 and blackcount==2){
        whitecount++;
      }
      if(checkallblack() and blackcount==2 and whitecount==2){
        blackcount++;
      }
      if(blackcount==3){
        mdrive(0,0);
        linefollowingcount2++;
        allblack2++;
        task7_2=1;

      }
    }      





  }
  
}

void gatedetector8(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin and calculate the distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound = 0.034 cm/us

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(1000);
  return distance;

}
//
void gatetimer8sonaside(){
  delay(1000);
  dstart = millis();
  while(ddifference < 4500){
    delay(100);
    while(measureDistance(trigPin, echoPin) < 10 && measureDistance(trigPin, echoPin) != 0){
    delay(100);
    }
    dend = millis();
    ddifference=dend - dstart;
    dstart = millis();
  }
  Serial.println("Done");
}
// distance eka change karanna
void gatetimer8sona1(){
  digitalWrite(30, HIGH);
  delay(1000);
  dstart = millis();
  while(ddifference < 4500){
    delay(100);
    while(measureDistance(trigPin, echoPin) < 10 && measureDistance(trigPin, echoPin) != 0){
    delay(100);
    }
    dend = millis();
    ddifference=dend - dstart;
    dstart = millis();
  }
  Serial.println("Done");
  digitalWrite(30, HIGH);
}

void gatedis8sona1(){
  digitalWrite(30, HIGH);
  delay(1000);
  long distance1 = measureDistance(trigPin1, echoPin1);
  Serial.print("Ultrasonic Sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  digitalWrite(30, LOW);
  delay(1000);
  return distance1;

}
void turnrighthalf(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = 920+200;
    target[1] = -920-200;
    while((posi[0]<target[0]-70-200)){
    // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,in1[k],in2[k]);
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}
void turnlefthalf(){
    mdrive(0,0);
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        posi[k]=0;
      }
    } 
    int target[NMOTORS];
    target[0] = -460-200;
    target[1] = 460+200;
    while((posi[1]<target[1]-200)){
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
    } 
  
      // loop through the motors
    for(int k = 0; k < NMOTORS; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      // signal the motor
      if(k==0){
        setMotor(dir,pwr,in1[k],in2[k]);
      }
      else{
        setMotor(dir,pwr,in1[k],in2[k]);
      }
    }

    for(int k = 0; k < NMOTORS; k++){
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
      Serial.println();
    }
    setMotor(1, 0 , 6, 7 );
    setMotor(1, 0 , 8 , 9 );

}