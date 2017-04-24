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
#define FLAME_PIN 14

Encoder rEncoder(ENCODER_R_A, ENCODER_R_B);
Encoder lEncoder(ENCODER_L_A, ENCODER_L_B);
L3G gyro;
LSM303 accel;

float WHEEL_DIAMETER = 0.07;
float WHEEL_SEPARATION = 0.17;
int TICKS_PER_REV = 1796; //Encoder ticks per rotation

int OdomWait = 3;
int OdomCount = 0;
double WCS[2] = {0, 0};

long EncoderVal[2] = {0, 0};
double DDis[2] = {0, 0};
long Time[2] = {0, 0};
double Vels[2] = {0, 0};

double G_Dt = 0.020;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long lastTime = 0;

double G_gain = 0.00493; // gyros gain factor for 250deg/sec
double gyro_x; //gyro x val
double gyro_y; //gyro x val
double gyro_z; //gyro x val
double gyro_xold; //gyro cummulative x value
double gyro_yold; //gyro cummulative y value
double gyro_zold; //gyro cummulative z value
double gerrx; // Gyro x error
double gerry; // Gyro y error
double gerrz; // Gyro 7 error

double last_gyro_x;
double last_gyro_y;

double v_gyro_x;
double v_gyro_y;

double A_gain = 0.00875; // gyros gain factor for 250deg/sec
double accel_x; //gyro x val
double accel_y; //gyro x val
double accel_z; //gyro x val
double accel_xold; //gyro cummulative x value
double accel_yold; //gyro cummulative y value
double accel_zold; //gyro cummulative z value
double aerrx; // Accel x error
double aerry; // Accel y error
double aerrz; // Accel 7 error

ros::NodeHandle nh;

// ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);

void setup() {
  nh.initNode();
  nh.advertise(Pub);
  odom_msg.linear.x = 1;
  odom_msg.linear.y = 2;
  odom_msg.angular.x = 3;
  odom_msg.angular.y = 4;
  odom_msg.angular.z = 5;
  Pub.publish(&odom_msg);

  pinMode(FLAME_PIN, INPUT);

  Wire.begin(); // i2c begin

  if (!gyro.init()) { // gyro init
    nh.logwarn("Failed to autodetect gyro type...");
    while (1);
  }
  gyro.enableDefault(); // gyro init. default 250/deg/s
  delay(1000);// allow time for gyro to settle
  nh.loginfo("Starting gyro calibration...");
  gyroZero();
  Accel_Init();
}

void loop() {
  nh.spinOnce();

  //first couple of times dont publish odom
  if (OdomCount > OdomWait) {
    odom_msg.linear.x = Vels[0];
    odom_msg.linear.y = Vels[1];
    odom_msg.angular.x = v_gyro_x;
    odom_msg.angular.y = v_gyro_y;
    odom_msg.angular.z = analogRead(FLAME_PIN);
    Pub.publish(&odom_msg);
  }
  else {
    OdomCount++;
  }

  doEncoders(0);
  doEncoders(1);
  complimentaryFilter();
  readGyro();

  delay(10);
}


void doEncoders(int M) {
  //if fist time in program return 0 and init time vars
  if (Time[0] == 0 && Time[1] == 0) {
    Time[0] = millis();
    Time[1] = millis();
  }

  //read encoder ticks
  if (M == 0) {
    EncoderVal[0] = lEncoder.read();
    lEncoder.write(0);
  }
  if (M == 1) {
    EncoderVal[1] = (rEncoder.read() * -1);
    rEncoder.write(0);
  }

  //differencial of time in seconds
  long T = millis();
  int DTime = T - Time[M];
  Time[M] = T;


  //diferential of distance in meters
  DDis[M] = ticksToMeters(EncoderVal[M]);

  //calculate short term measured velocity
  double EVel = (DDis[M] / DTime) * 1000;

  //save to publish to /ard_odom
  Vels[M] = EVel;
}

void gyroZero() {
  // takes 200 samples of the gyro
  for (int i = 0; i < 200; i++) {
    gyro.read();
    gerrx += gyro.g.x;
    gerry += gyro.g.y;
    gerrz += gyro.g.z;
    delay(20);
  }
  gerrx = gerrx / 200; // average reading to obtain an error/offset
  gerry = gerry / 200;
  gerrz = gerrz / 200;
}

void readGyro() {
  gyro.read(); // read gyro

  gyro_x = (double)(gyro.g.x - gerrx) * G_gain; // offset by error then multiply by gyro gain factor
  gyro_y = (double)(gyro.g.y - gerry) * G_gain;
  gyro_z = (double)(gyro.g.z - gerrz) * G_gain;

  gyro_x = gyro_x * G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y * G_Dt;
  gyro_z = gyro_z * G_Dt;

  gyro_x = (gyro_x / 180) * PI;
  gyro_y = (gyro_y / 180) * PI;

  gyro_x += gyro_xold; // add the displacment(rotation) to the cumulative displacment
  gyro_y += gyro_yold;
  gyro_z += gyro_zold;

  gyro_xold = gyro_x ; // Set the old gyro angle to the current gyro angle
  gyro_yold = gyro_y ;
  gyro_zold = gyro_z ;
}

void Accel_Init()
{
  accel.init();
  accel.enableDefault();

  switch (accel.getDeviceType())
  {
    case LSM303::device_D:
      accel.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      accel.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accel.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

void accelZero() {
  //I found this to be more problematic than it was worth.
  //not implemented
  // takes 100 samples of the accel
  for (int i = 0; i < 100; i++) {
    gyro.read();
    aerrx += accel.a.x >> 4;
    aerry += accel.a.y >> 4;
    aerrz += accel.a.z >> 4;
    delay(10);
  }
  aerrx = gerrx / 100; // average reading to obtain an error/offset
  aerry = gerry / 100;
  aerrz = gerrz / 100;
}


// Reads x,y and z accelerometer registers
void readAccel()
{
  accel.readAcc();

  accel_x = accel.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  accel_y = accel.a.y >> 4;
  accel_z = accel.a.z >> 4;

  // accelerations in G
  accel_x = (accel_x / 256);
  accel_y = (accel_y / 256);
  accel_z = (accel_z / 256);
}

void complimentaryFilter() {
  if (lastTime == 0) {
    lastTime = millis();
  }

  long curTime = millis();
  long d_time = curTime - lastTime;

  readGyro();
  readAccel();

  double x_Acc, y_Acc;
  double magnitudeofAccel = (abs(accel_x) + abs(accel_y) + abs(accel_z));
  if (magnitudeofAccel < 6 && magnitudeofAccel > 1.2)
  {
    x_Acc = atan2(accel_y, accel_z);
    gyro_x = gyro_x * 0.995 + x_Acc * 0.08;

    y_Acc = atan2(accel_x, accel_z);
    gyro_y = gyro_y * 0.98 + y_Acc * 0.02;
  }

  v_gyro_x = getRadsPerSec(d_time, gyro_x, last_gyro_x);
  v_gyro_y = getRadsPerSec(d_time, gyro_y, last_gyro_y);

  last_gyro_x = gyro_x;
  last_gyro_y = gyro_y;
  lastTime = curTime;
}

double getRadsPerSec(long deltaMillis, double curRads, double lastRads) {
  return ((curRads - lastRads) / deltaMillis) * 1000;
}

double ticksToMeters(long ticks) {
  return (ticks * ((PI * WHEEL_DIAMETER) / TICKS_PER_REV));
}
