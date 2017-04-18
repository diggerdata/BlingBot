#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

// encoder pins
#define ENCODER_R_A 0
#define ENCODER_R_B 1
#define ENCODER_L_A 2
#define ENCODER_L_B 3

#define IMU

Encoder rEncoder(encrA, encrB);
Encoder lEncoder(enclA, enclB);
L3G gyro;
LSM303 accel;

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

float G_Dt=0.020;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpose timer
long timer1=0;
long timer2=0;

float G_gain=.0049; // gyros gain factor for 250deg/sec
float gyro_x; //gyro x val
float gyro_y; //gyro x val
float gyro_z; //gyro x val
float gyro_xold; //gyro cummulative x value
float gyro_yold; //gyro cummulative y value
float gyro_zold; //gyro cummulative z value
float gerrx; // Gyro x error
float gerry; // Gyro y error
float gerrz; // Gyro 7 error

ros::NodeHandle nh;

// ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);

void setup() {
  nh.initNode();
  nh.advertise(Pub);
  odom_msg.linear.x = 1;
  odom_msg.linear.y = 2;
  odom_mst.angular.z = 3;
  Pub.publish(&odom_msg);

  Wire.begin(); // i2c begin

  if (!gyro.init()) { // gyro init
    while (1);
  }

  timer = millis(); // init timer for first reading
  gyro.enableDefault(); // gyro init. default 250/deg/s
  delay(1000); // allow time for gyro to settle

  gyroZero();
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

  delay(10);
}

void gyroZero(){
// takes 200 samples of the gyro
  for(int i =0;i<200;i++) {
    gyro.read();
    gerrx+=gyro.g.x;
    gerry+=gyro.g.y;
    gerrz+=gyro.g.z;
    delay(20);
  }

  gerrx = gerrx/200; // average reading to obtain an error/offset
  gerry = gerry/200;
  gerrz = gerrz/200;

  Serial.println(gerrx); // print error vals
  Serial.println(gerry);
  Serial.println(gerrz);
}

void readGyro() {
  gyro.read(); // read gyro
  timer=millis(); //reset timer
  gyro_x=(float)(gyro.g.x-gerrx)*G_gain; // offset by error then multiply by gyro gain factor
  gyro_y=(float)(gyro.g.y-gerry)*G_gain;
  gyro_z=(float)(gyro.g.z-gerrz)*G_gain;

  gyro_x = gyro_x*G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y*G_Dt;
  gyro_z = gyro_z*G_Dt;

  gyro_x +=gyro_xold; // add the displacment(rotation) to the cumulative displacment
  gyro_y += gyro_yold;
  gyro_z += gyro_zold;

  gyro_xold=gyro_x ; // Set the old gyro angle to the current gyro angle
  gyro_yold=gyro_y ;
  gyro_zold=gyro_z ;
}

void doEncoders(int M) {
  //if fist time in program return 0 and init time vars
  if(Time[0]==0 && Time[1] == 0){
    Time[0] = millis();
    Time[1] = millis();
  }

  //read encoder ticks
  if(M == 0){
    EncoderVal[0] = lEncoder.read();
    lEncoder.write(0);
  }
  if(M == 1){
    EncoderVal[1] = (rEncoder.read()*-1);
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
