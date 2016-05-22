#include "abdrive.h"
#include "ping.h" 
#include "simpletools.h"
#include "mylib.h"

#define SET_LEFT 12
#define SET_RIGHT 12
#define WAYPOINT_FREQUENCY 4
#define MAP_WIDTH 2
#define MAP_X_SCALING 8
#define MAP_HEIGHT 16
#define MAZE_SQUARE_TICKS 117
#define MAZE_SQUARE_CM 38

#define PING_PIN 8

int irLeft, irRight;
int irLeftOld, irRightOld;
float speedModifier = 2.0f;

int distance=10000;

int left,right,leftold,rightold;

float turnangle_old,robot_radius;

int xPosition,yPosition;
char robotDir;

int accError = 0;

char map[MAP_WIDTH][MAP_HEIGHT];
char visited[MAP_WIDTH][MAP_HEIGHT];
char racing = 0;

/**
 * Gets encoder ticks and computes the global variables required to know the robot's position and angle
 */
void dead_reckoning(){
  /*drive_getTicks(&left, &right);                                    //gets ticks
    
    int deltaRight=right-rightold;                                  //gets the actual movement
    int deltaLeft=left-leftold;                                     //on this one move
    
    float turnangle_new=(deltaRight-deltaLeft)/wheelDistance;       //calculate turn angle or "fi"
    
    float delta_y, delta_x;

    if(deltaLeft - deltaRight != 0){
      float leftradius,rightradius;
      leftradius=deltaLeft/turnangle_new;                           //calculate left and right radii
      rightradius=deltaRight/turnangle_new;
      robot_radius=(leftradius+rightradius)/2;                      //calculate overall robot radius

      delta_y = robot_radius * sin(turnangle_new);
      delta_x = robot_radius * cos(turnangle_new) - robot_radius;

      float sinv = sin(turnangle_old);
      float cosv = cos(turnangle_old);
      y_position += delta_y*sinv - delta_x*cosv;
      x_position += delta_x*sinv + delta_y*cosv;
    }
    else{															                                     //Going forward
      turnangle_new = 0.0f;
      delta_y = deltaLeft;											                          //Should be the same as deltaRight
		
      y_position += sin(turnangle_old)*delta_y;
      x_position += cos(turnangle_old)*delta_y;
    }
	
    rightold=right;                                                 //memory update
    leftold=left;
    
    if(waypointIteration>=0) waypointIteration++;
    if(waypointIteration>=WAYPOINT_FREQUENCY){
      Waypoint *w = (Waypoint*)malloc(sizeof(Waypoint));
      w->x_pos = x_position;
      w->y_pos = y_position;
      w->next = WaypointHead;
      WaypointHead = w;
      waypointIteration = 0;
    }
    
    turnangle_old+=turnangle_new;
    if(turnangle_old > PI) turnangle_old -= 2*PI;
    if(turnangle_old < -PI) turnangle_old += 2*PI;*/
}

void initMap(){
  int i;
  for(i=0;i<MAP_WIDTH*MAP_HEIGHT;i++){
    if(i%MAP_HEIGHT == 0 || i%MAP_HEIGHT == MAP_HEIGHT-1) map[i/MAP_HEIGHT][i%MAP_HEIGHT] = 0xff;
    else if(i/MAP_HEIGHT == 0) map[i/MAP_HEIGHT][i%MAP_HEIGHT] = 0x80;
    else if(i/MAP_HEIGHT == MAP_WIDTH-1) map[i/MAP_HEIGHT][i%MAP_HEIGHT] = 0x01;
    else map[i/MAP_HEIGHT][i%MAP_HEIGHT] = 0;
  }
  for(i=0;i<MAP_WIDTH*MAP_HEIGHT;i++){
    visited[i/MAP_HEIGHT][i%MAP_HEIGHT]=0;
  }
}

void setMaplikeCoord(char a[MAP_WIDTH][MAP_HEIGHT], int x, int y, char val){
  if(val==1) a[x/MAP_X_SCALING][y] |= 0x80 >>(x%MAP_X_SCALING);
  if(val==0) a[x/MAP_X_SCALING][y] &= 0xff ^ (0x80>>(x%MAP_X_SCALING));
}

char getMaplikeCoord(char a[MAP_WIDTH][MAP_HEIGHT], int x, int y){
  return (a[x/MAP_X_SCALING][y] & (0x80>>(x%MAP_X_SCALING))) != 0;
}  

void setMapCoord(int x, int y, char val){
  setMaplikeCoord(map, x, y, val);
}

void visitCurrentPlace(){
  setMaplikeCoord(visited, xPosition*2+1, yPosition*2+1, 1);
}

void adjustNormal(){
  /*int dDist = 0; int oldDDist = 0;
  int ddDist = 0;
  int dir = 1;
  int dist = ping_cm(PING_PIN);  int oldDist;
  int modi;
  int counter=0;
  while(modi<=16){
    oldDist = dist; dist = ping_cm(PING_PIN);
    oldDDist = dDist; dDist = dist-oldDist;
    ddDist = dDist - oldDDist;

    printf("Dist: %d, dDist: %d, ddDist: %d\n", dist, dDist, ddDist);

    if(dDist>0 && ddDist>0) dir=-dir;
    modi = 2<<(counter/20);
    if(dir==1) drive_speed(32/modi,-32/modi);
    else if(dir==-1) drive_speed(-32/modi,32/modi);
    counter++;
    pause(50);
  }
  drive_ramp(0,0);*/
  /*int dir = 1;
  int dist = ping_cm(PING_PIN);
  int lowestDist = dist;
  int modi = 2;
  int phase = 0;
  while(1){
    dist = ping_cm(PING_PIN); printf("Dist: %d\n", dist);
    if(dist>lowestDist && phase==0) {dir=-dir; lowestDist=dist; phase=1;}
    if(dist<lowestDist && phase==0) {lowestDist=dist; phase=1;}
    if(dist>lowestDist && phase==1) {dir=-dir; phase=2;}
    if(dist<lowestDist && phase==1) {lowestDist=dist;}
    if(dist==lowestDist && phase==2) break;

    if(dir==1) drive_speed(32/modi,-32/modi);
    else if(dir==-1) drive_speed(-32/modi,32/modi);
    pause(50);
  }
  drive_ramp(0,0);*/
}

void turnRobotTo(int dir){
  int degree = (dir-robotDir)*90;
  robotDir=dir;
  if(degree == 0) return;
  turnInPlaceDeg(degree);
}

char getVisitedInDir(char dir){
  switch(dir){
  case 0: return getMaplikeCoord(visited, xPosition*2+1, yPosition*2+3);
  case 1: return getMaplikeCoord(visited, xPosition*2+3, yPosition*2+1);
  case 2: return getMaplikeCoord(visited, xPosition*2+1, yPosition*2-1);
  case 3: return getMaplikeCoord(visited, xPosition*2-1, yPosition*2+1);
  default: return 0;
  }
}

char getWallInDir(char dir){
  switch(dir){
  case 0: return getMaplikeCoord(map, xPosition*2+1, yPosition*2+2);
  case 1: return getMaplikeCoord(map, xPosition*2+2, yPosition*2+1);
  case 2: return getMaplikeCoord(map, xPosition*2+1, yPosition*2);
  case 3: return getMaplikeCoord(map, xPosition*2, yPosition*2+1);
  default: return 0;
  }
}

void scanSurroundings(){
  int dists[4], i;
  int distAvg = MAZE_SQUARE_CM/2-6, surrWalls = 0;
  char origDir = robotDir;
  for(i=0;i<4;i++){
    int h = i;
    i = (i+origDir) % 4;
    if(getVisitedInDir(i) && !getWallInDir(i)) { dists[i]=MAZE_SQUARE_CM*2; i=h; continue; }
    turnRobotTo(i);
    dists[i] = ping_cm(PING_PIN);
    if(dists[i]<MAZE_SQUARE_CM){
      /*distAvg += dists[i];*/ surrWalls++;
      adjustNormal();
      if(dists[i] < distAvg){
        drive_goto((dists[i]-distAvg)/distancePerTick*10, (dists[i]-distAvg)/distancePerTick*10);
        dists[i] = ping_cm(PING_PIN);
      }

      if(i==0) setMapCoord(xPosition*2+1, yPosition*2+2, 1);
      if(i==1) setMapCoord(xPosition*2+2, yPosition*2+1, 1);
      if(i==2) setMapCoord(xPosition*2+1, yPosition*2, 1);
      if(i==3) setMapCoord(xPosition*2, yPosition*2+1, 1);
    }
    i = h;
  }
  
  /*distAvg /= surrWalls;*/ distAvg = MAZE_SQUARE_CM/2-6;
  for(i=3;i>=0;i--){
    if(dists[i] < MAZE_SQUARE_CM && dists[i] > distAvg+3 && dists[(i+2)%4] > MAZE_SQUARE_CM){
      turnRobotTo(i);
      drive_goto((dists[i]-distAvg)/distancePerTick*10, (dists[i]-distAvg)/distancePerTick*10);
    }
  }
}

void printMaze(){ //For debugging purposes
  /*int i;
  for(i=0;i<MAP_WIDTH*MAP_HEIGHT*MAP_X_SCALING*2;i++){
    char p = (i%(MAP_HEIGHT*2)>=MAP_HEIGHT) ?
             (visited[i/MAP_HEIGHT/MAP_X_SCALING/2][i%MAP_HEIGHT] & (0x80>>((i/MAP_HEIGHT/2) % MAP_X_SCALING))):
             (map[i/MAP_HEIGHT/MAP_X_SCALING/2][i%MAP_HEIGHT] & (0x80>>((i/MAP_HEIGHT/2) % MAP_X_SCALING)));
    if(p!=0) p=1;
    if(p==0) printf(" ");
    if(p==1) printf("#");
    if(i%(MAP_HEIGHT*2)==MAP_HEIGHT*2-1) putchar('\n');
  }*/
}

char dijkstraToMe(int x, int y){
  unsigned short int costMap[MAP_WIDTH*MAP_X_SCALING][MAP_HEIGHT];
  char prev[MAP_WIDTH*MAP_X_SCALING][MAP_HEIGHT];
  char visited[MAP_WIDTH][MAP_HEIGHT];
  int i;
  for(i=0;i<MAP_WIDTH*MAP_HEIGHT*MAP_X_SCALING;i++) costMap[i/MAP_HEIGHT][i%MAP_HEIGHT]=0xffff;
  for(i=0;i<MAP_WIDTH*MAP_HEIGHT;i++) visited[i/MAP_HEIGHT][i%MAP_HEIGHT]=0;
  costMap[x*2+1][y*2+1]=0;
  
  int curX = x*2+1, curY = y*2+1;
  while(1){
    setMaplikeCoord(visited, curX, curY, 1);
    if(curX==xPosition*2+1 && curY==yPosition*2+1) break;
    for(i=0;i<4;i++){
      int nX=curX, nY=curY;
      if(i==0) nY+=2;
      if(i==1) nX+=2;
      if(i==2) nY-=2;
      if(i==3) nX-=2;
      if(getMaplikeCoord(map,(nX+curX)/2,(nY+curY)/2)) continue;
      unsigned short int altCost = costMap[curX][curY] + 1;
      if(altCost < costMap[nX][nY]){
        costMap[nX][nY] = altCost;
        prev[nX][nY]=(i+2)%4;
      }
    }
    
    //Finding minimum cost node, could be implemented using a priority queue but adds a lot of code.
    unsigned short int minCost = 0xffff;
    for(i=0;i<MAP_WIDTH*MAP_HEIGHT*MAP_X_SCALING;i++){
      if(getMaplikeCoord(visited,i/MAP_HEIGHT,i%MAP_HEIGHT)) continue;
      if(costMap[i/MAP_HEIGHT][i%MAP_HEIGHT]<minCost){
        minCost = costMap[i/MAP_HEIGHT][i%MAP_HEIGHT];
        curX = i/MAP_HEIGHT; curY = i%MAP_HEIGHT;
      }        
    }
    if(minCost == 0xffff) {prev[xPosition*2+1][yPosition*2+1]=0xff; break;}
  }
  
  return prev[curX][curY];
}

void driveForward(){
  int left, right, leftStart, rightStart;
  int irLeft, irRight, irLeftOld, irRightOld;

  drive_getTicks(&leftStart, &rightStart);
  calculateIR(&irLeft, &irRight, &irLeftOld, &irRightOld);
  left = leftStart, right = rightStart;
  drive_setRampStep(4*speedModifier);

  while((left < leftStart+MAZE_SQUARE_TICKS || right < rightStart+MAZE_SQUARE_TICKS )
   && ping_cm(PING_PIN)>MAZE_SQUARE_CM/2-6){
    drive_getTicks(&left, &right);

    calculateIR(&irLeft, &irRight, &irLeftOld, &irRightOld);
    //printf("lOld: %d, rOld: %d, l: %d, r: %d\n", irLeftOld, irRightOld, irLeft, irRight);
    int dl = irLeft-irLeftOld;
    int dr = irRight-irRightOld;

    int correcterLeft = (irRight-irLeft)*1; correcterLeft=0;
    int correcterRight = (irLeft-irRight)*1; correcterRight=0;

    if((robotDir%2==0)&&!(getMaplikeCoord(map,xPosition*2,yPosition*2+1)&&getMaplikeCoord(map,xPosition*2+2,yPosition*2+1)))
      correcterLeft=correcterRight=0;
    if((robotDir%2==1)&&!(getMaplikeCoord(map,xPosition*2+1,yPosition*2)&&getMaplikeCoord(map,xPosition*2+1,yPosition*2+2)))
      correcterLeft=correcterRight=0;
    if(left-leftStart>MAZE_SQUARE_TICKS/3 || right-rightStart>MAZE_SQUARE_TICKS/3)
      correcterLeft=correcterRight=0;
    correcterLeft += (dr - dl)*2;
    correcterRight += (dl - dr)*2;
    if(irLeft<5 && left-leftStart<MAZE_SQUARE_TICKS*3/4) {correcterLeft+=4; correcterRight+=-4;}
    if(irRight<5 && left-leftStart<MAZE_SQUARE_TICKS*3/4) {correcterLeft+=-4; correcterRight+=4;}
    /*if(irLeft>19) {correcterLeft+=-4; correcterRight+=4;}
    if(irRight>19) {correcterLeft+=4; correcterRight+=-4;}*/
    correcterLeft *= speedModifier;
    correcterRight *= speedModifier;

    drive_rampStep(32*speedModifier+correcterLeft, 32*speedModifier+correcterRight);
  }
  //drive_goto(MAZE_SQUARE_TICKS,MAZE_SQUARE_TICKS);
  if(!racing) drive_ramp(0,0);
  xPosition+=(robotDir%2)*(2-robotDir);
  yPosition+=((robotDir+1)%2)*(1-robotDir);
}

void mazeTo(int x, int y){
  while(xPosition != x || yPosition != y){
    visitCurrentPlace();
    scanSurroundings();
    
    char dir = dijkstraToMe(x,y);
    printMaze();
    if(dir==0xff) {/*printf("Fatal error! No path - lost maybe?");*/ return;}
    turnRobotTo(dir);
    driveForward();
  }
}

void raceTo(int x, int y){
  speedModifier = 2.0f;
  while(xPosition != x || yPosition != y){    
    char dir = dijkstraToMe(x,y);
    if(dir==0xff) {/*printf("Fatal error! No path - lost maybe?");*/ return;}
    if(dir!=robotDir) drive_ramp(0,0);
    turnRobotTo(dir);
    driveForward();
  }
}

void mapOutMaze(){  
  int i;
  mazeTo(4,2);
  mazeTo(4,5);
  mazeTo(1,5);
  mazeTo(1,1);
  
  printMaze();
}

int main()
{
  //LED setup
  int i;
  dac_ctr_stop();
  low(26); low(27);
  
  //Variable setup
  xPosition=yPosition=1; robotDir = 0;
  turnangle_old=PI/2;
  drive_getTicks(&left, &right);
  rightold = right;
  leftold = left;
  
  pause(500);
  //Main loop
  initMap();
  mapOutMaze();

  turnRobotTo(0);
  for(i=0;i<3;i++){pause(166); high(26); pause(166); low(26);}
  pause(1000);
  racing = 1;
  raceTo(4,5);
}
