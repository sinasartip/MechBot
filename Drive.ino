#include <mechbotShield.h>
#include <avr/io.h>
#include "MEC733_I_O.h"

// global variables
int Lms, Rms, measure, derivative, error, last_error;
int turnSpeed = 900, maze_speed = 500, forward_stopping_distance = 200, goal = 350; // forward_stopping_distance = 180, goal = 300; SPEED 700
float error_scale = 1.5;
int kp = 5;
int kd = 25;
//5 15
int t = 300;
int surface_changes = 0;
bool surface_previous;
int distF, distFp = 0;

/*  
 * play around with right distance sensor angle, maze_speed, kp
 * get some numbers for forward_stopping_distance, goal
 */


int main(void)
{
  initSoftSerial();
  setLCDBackLight(147);
  clrLCD();
  moveLCDCursor(0);
  initMotor();
  initADC();

  portSetup();
  waitForBumpStart();
  clrLCD(); moveLCDCursor(0); lcdPrint("following line!");
  quickForward(300);
  followLine();
  quickForward(1000);
  clrLCD(); moveLCDCursor(0); lcdPrint("Going through maze");
  navigateMaze();
  clrLCD(); moveLCDCursor(0); lcdPrint("Mission accomplished");
  quickForward(500);
  motor(0,0);
}

void navigateMaze(void)
{
  bool check = 0;
  
  while (1)
  {
    check = hugRight(); // hugRight returns 1 if out of the maze
    if (check)
    { return; }
    delay_ms(t);
    // turn away from the wall in front
    if( analog(4) < 200 ) // if path clear to the left, turn left
    { turnLeft(); delay_ms(t); }
    else
    { turnAround(); delay_ms(t); }
  }
}

bool hugRight(void)
{
  int bump1, bump2, bump3;
  int turn = 0; last_error = 0; distFp = 0;

  while (1)
  {
    // stop and exit with 0 if there is a wall in front
    distF = int(analog(5)+distFp)/2;
    distFp = distF;
    if (distF > forward_stopping_distance)
    { motor(0,0); return(0); }
    
    // exit with 1 if out of maze
    measure = analog(6);
    if ( trackSurface() )
    {
      if( surface_changes > 2 )
      {
        if ( measure < 50 && analog(4) < 50 )
          { motor(0,0); return(1); }
      }
    }

    // keep adjusting speed based on distance from right wall
    error = measure - goal;
    derivative = error - last_error;
    last_error = error;
    turn = int( kp*error + kd*derivative );
    Lms = constrain( maze_speed - turn, -1000, 1000 );
    Rms = constrain( maze_speed + turn, -1000, 1000 );
    motor(Lms,Rms);
  }
}

bool trackSurface(void)
{
  uint16_t line_1, line_2, line_3, line_4, mazeThresh = 900;
  bool surface;
  line_1 = analog(0);
  line_2 = analog(1);
  line_3 = analog(2);
  line_4 = analog(3);
  if (line_1 > mazeThresh || line_2 > mazeThresh || line_3 > mazeThresh || line_4 > mazeThresh) // in maze
  {
      surface = 0;
  }
  else { // out of maze
      surface = 1;
  }
  if (surface != surface_previous) {
    surface_changes ++;
    if ( surface_changes > 1 )
    { goal = 300; }
    surface_previous = surface;
  }
  return (surface);
}

void turnLeft(void)
{ encoderDrive(15, -turnSpeed, turnSpeed); }

void turnAround(void)
{ encoderDrive(32, -turnSpeed, turnSpeed); } //31

void encoderDrive(int enReadings, int velL, int velR)
{
  int left_en_total=0, right_en_total=0, left_en_cur, right_en_cur, left_en_pre, right_en_pre;
  motor(velL,velR);
  
  while (1)  
  {
    left_en_cur=PINC&(1<<PINC2);
    right_en_cur=PINC&(1<<PINC3);

    if(left_en_cur!=left_en_pre) 
    { left_en_total++; left_en_pre = left_en_cur; }

    if(right_en_cur!=right_en_pre)
    { right_en_total++; right_en_pre = right_en_cur; }

    if ((left_en_total>=enReadings)&&(right_en_total>=enReadings))
    { motor(0,0); return; }
    
    delay_ms(10);
  }
}

void quickForward(int t)
{
  motor(800,800); //a quick straight line forward
  delay_ms(t);
}

void followLine(void)
{
  uint16_t line_1, line_2, line_3, line_4, thresh = 750, floorR = 600, lineR = 850, t = 10;
  int turn, line_speed = 600, velF = line_speed, velR = -velF;
  
  motor(line_speed,line_speed);
  delay_ms(500);

  while (1)
  {
    line_1 = analog(0);
    line_2 = analog(1);
    line_3 = analog(2);
    line_4 = analog(3);
    
    if (line_2 > thresh & line_3 > thresh) 
    { motor(velF, velF); }
    else if (line_2 < thresh & line_4 > thresh) 
    { motor(velF, 0); }
    else if (line_1 > thresh & line_3 < thresh) 
    { motor(0, velF); }
    else if (line_1 < thresh & line_4 > thresh) 
    { motor(velF, velR); }
    else if (line_1 > thresh & line_4 < thresh) 
    { motor(velR, velF); }
    else
    { motor(velF, velF); }
    
    if (line_1 > thresh && line_2 > thresh && line_3 > thresh && line_4 > thresh)
    { motor(0,0);
      clrLCD(); moveLCDCursor(0); lcdPrint("end of the line!");
      delay_ms(3000);
      return; }
      
    //delay_ms(30);
  }
}

void waitForBumpStart(void)
{
  int bump1, bump2, bump3;
  clrLCD(); moveLCDCursor(0); lcdPrint("press bump switch to start");

  while (1)
  {
    bump1 = PIND & (1 << PIND3);
    bump2 = PIND & (1 << PIND4);
    bump3 = PIND & (1 << PIND5);
    if ((bump1 == 0) | (bump2 == 0) | (bump3 == 0))
    { delay_ms(500); return; }
  }
}

void portSetup(void)
{
  // port D pins 3, 4 and 5 for bumper switches
  DDRD = DDRD & (~((1<<DDRD3)|(1<<DDRD4)|(1<<DDRD5)));
  PORTD = PORTD | ((1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5));

  // port C pins 2 and 3 for encoder readings
  DDRC = DDRC & (~((1<<DDRC2)|(1<<DDRC3)));
  PORTC = PORTC | ((1<<PORTC2)|(1<<PORTC3));
}
