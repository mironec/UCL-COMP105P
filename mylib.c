#include "mylib.h"

#define IR_STEP 8

const int DO = 22, CLK = 23, DI = 24, CS = 25;              // SD card pins on Propeller BOE
const float widthOfRobotF = 105.8f;                         // In mm
const int widthOfRobot = 106;                               // In mm and int
const float distancePerTick = 3.25f;                        // In mm
const float wheelDistance = 32.5538f;                       // In encoder ticks

/*char* itos(int x) {
	char* c = (char *)malloc(sizeof(char) * 10);
	int i;
 char negative = 0;
 if(x<0){
   x = -x;
   negative = 1;
 }   
	
	for(i=8;i>=0;i--){
		c[i]='0' + (x%10);
		x/=10;
	}
	c[9] = '\0';
	
  if(negative){c[0]='-'; i=1;}
  else{i=0;}
	for(;i<9 && c[i]=='0';i++){
		int j;
		for(j=i;c[j]!='\0';j++){
			c[j]=c[j+1];
		}
		i--;
	}
	
	return c;
}*/

double msin(double x){
	return (x>PI)?msin(x-2*PI):(x<0?-msin(-x):(x<PI/2?(x-pow(x,3)/6+pow(x,5)/120):(-x+PI+pow(x-PI,3)/6 - pow(x-PI,5)/120)));
}

double mcos(double x){
  return msin(PI/2-x);
}

double masin(double x){
  return (x<0)?-masin(-x):((x<0.8)?(x+pow(x,3)/6+pow(x,5)/40*3):(1.176+13*(x-0.923)/5 + 1014/125*pow(x-0.923,2)));
}


/**
 * Simulates distance sensing for the IR sensors via decreasing the voltage the emitters get and counting the number of times the
 * sensors sense something
 */
void calculateIR(int * irLeft, int * irRight, int * irLeftOld, int * irRightOld){
  *irLeftOld = *irLeft;
  *irRightOld = *irRight;
  /*int el = 160/IR_STEP, bl = 0, er = 160/IR_STEP, br = 0;
  while(el-bl>1 || er-br>1){
    int dacVal = (el+bl)/2;
    dac_ctr(26, 0, 160-dacVal*IR_STEP);
    dac_ctr(27, 1, 160-dacVal*IR_STEP);
    freqout(11, 1, 38000);
    if(input(10)) bl=dacVal;
    else el=dacVal;
    
    dacVal = (er+br)/2;
    dac_ctr(26, 0, 160-dacVal*IR_STEP);
    dac_ctr(27, 1, 160-dacVal*IR_STEP);
    freqout(1, 1, 38000);
    if(input(2)) br=dacVal;
    else er=dacVal;
  }
  *irLeft=bl;
  *irRight=br;*/
  *irLeft=*irRight=0;
  for(int dacVal = 0; dacVal < 160; dacVal += 8) {
    dac_ctr(26, 0, dacVal);
    freqout(11, 1, 38000);
    *irLeft += input(10);

    dac_ctr(27, 1, dacVal);
    freqout(1, 1, 38000);
    *irRight += input(2);
  }
}

void turnInPlace(double angle){
  double dist = wheelDistance/2*angle;
  int ticks = (int)(dist);
  static double overshoot = 0;
  
  overshoot += dist-(double)ticks;

  if(overshoot>1) {overshoot-=1; ticks++;}

  //ticks+=rand()%(ticks>>4)-(ticks>>5);

  drive_goto(ticks, -ticks);
}

void turnInPlaceNoOvershoot(double angle){
  double dist = wheelDistance/2*angle;
  int ticks = (int)(dist);
  static double overshoot = 0;
  
  overshoot += dist-(double)ticks;

  if(overshoot>1) {overshoot-=1; ticks++;}
  
  /*int left,right; drive_getTicks(&left, &right);
  int leftStart = left, rightStart = right;
  int lowestDist = 0xffff;
  while(1){
    int dist = ping_cm(8);
    drive_getTicks(&left, &right);
    if(dist<lowestDist&&
    ((double)abs(right-rightStart)+abs(left-leftStart))/ticks>1.4) lowestDist=dist;
    if(dist>lowestDist&&
    ((double)abs(right-rightStart)+abs(left-leftStart))/ticks>1.5) break;
    if(abs(left-leftStart)>=ticks && abs(right-rightStart)>=ticks) break;
    drive_rampStep(angle>0?32:-32,angle>0?-32:32);
  }
  drive_speed(0,0);*/
  drive_goto(ticks, -ticks);
}

void turnInPlaceDeg(double angle){
  turnInPlace(angle/180*PI);
}

void turnInPlaceDegNoOvershoot(double angle){
  turnInPlaceNoOvershoot(angle/180*PI);
}
