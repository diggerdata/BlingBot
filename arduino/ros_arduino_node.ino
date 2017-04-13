#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>

//THESE ARE THE ENCODER READ PINS
#define encrA 0
#define encrB 1
#define enclA 2
#define enclB 3

Encoder rEncoder(encrA, encrB);
Encoder lEncoder(enclA, enclB);

double curTime2 = 0;
double prevTime2 = 0;
double timeInterval2 = 0;
double movement2 = 0;
double prevPos2;

float WHEEL_DIAMETER = 0.07;
float WHEEL_SEPARATION = 0.17;
int TICKS_PER_REV = 1796; //Encoder ticks per rotation 

int OdomWait = 3;
int OdomCount = 0;
double WCS[2] = {0,0};

long EncoderVal[2] = {0,0};
double DDis[2] = {0,0};
long Time[2] = {0,0};
double Vels[2] = {0,0};

ros::NodeHandle nh;

////ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);

void setup() {
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(Pub);
  odom_msg.linear.x = 1;
  odom_msg.linear.y = 2;
  Pub.publish(&odom_msg);

  //nh.getParam("/serial_node/WheelSeparation", &WHEEL_SEPARATION, 1);
  //nh.getParam("/serial_node/WheelDiameter", &WHEEL_DIAMETER, 1);
}

void loop() {
  nh.spinOnce();

  //first couple of times dont publish odom
  if (OdomCount > OdomWait) {
    odom_msg.linear.x = Vels[0];
    odom_msg.linear.y = Vels[1];
    Pub.publish(&odom_msg);
  }
  else {
    OdomCount++;
  }

  doEncoders(0);
  doEncoders(1);

  delay(3);
}

void doEncoders(int M) {
  //if fist time in program return 0 and init time vars
  if(Time[0]==0 && Time[1] == 0){
    Time[0] = millis();
    Time[1] = millis();
  }

  //read encoder ticks
  if(M == 0){
    EncoderVal[0] = lEncoder.read()*-1;
    lEncoder.write(0);
  }
  if(M == 1){
    EncoderVal[1] = (rEncoder.read());
    rEncoder.write(0);
  }

  //differencial of time in seconds
  long T = millis();
  int DTime = T-Time[M];
  Time[M] = T;


  //diferential of distance in meters
  DDis[M] = ticksToMeters(EncoderVal[M]);
  
  //calculate short term measured velocity
  double EVel = (DDis[M]/DTime)*1000;
  
  //save to publish to /ard_odom
  Vels[M] = EVel;
}

double ticksToMeters(long ticks) {
  return (ticks * ((PI*WHEEL_DIAMETER) / TICKS_PER_REV));
}

