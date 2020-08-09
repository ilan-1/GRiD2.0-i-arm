#include <ros.h>
#include <traversal/WheelRpm.h>

//Sensors
#define LS 2      
#define RS 3   
   
//Motors
#define LM1 4       
#define RM1 6       
#define RM2 7       

//Initialise
int mode = 0, vel = 0;
ros::NodeHandle nh;
ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);

//Function to find motor values
void roverMotionCallback(const traversal::WheelRpm& RoverRpm)
{
  
  mode = RoverRpm.mode;

  //When picking objects
  if(mode == 0)
  {
  vel = RoverRpm.omega;
  analogWrite(LM1, vel);
  analogWrite(LM2, vel);
  analogWrite(RM1, vel);
  analogWrite(RM2, vel);
  }

  //For line following from one point to another
  if(mode == 1)
  {
  if(digitalRead(LS) && digitalRead(RS))     // Move Forward
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  
  if(!(digitalRead(LS)) && digitalRead(RS))     // Turn right
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  }
  
  if(digitalRead(LS) && !(digitalRead(RS)))     // turn left
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
  
  if(!(digitalRead(LS)) && !(digitalRead(RS)))     // stop
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
  }
}



void setup()
{
  nh.initNode();
  nh.subscribe(locomotion_sub);
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
}





void loop() {
  nh.spinOnce();

}
