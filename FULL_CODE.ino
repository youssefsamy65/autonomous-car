#include "MPU9250.h"
#include <PID_v1.h>
#include <math.h>
//--------------------PID_VARIABLES-------------------
  double Setpoint, Input, Output, Kp = 7.22, Ki = 0.2, Kd = 0.51, error, sum_error, prev_error, sum_squared_error;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  double delta_kp = 0.005;
  double delta_kd = 0.001;
  double delta_ki = 0.001;
  int num_no_improvement_kp=0;
  int num_no_improvement_ki=0;
  int num_no_improvement_kd=0;
  int num_no_improvement=0;
    double prev_itae ;
//---------------------IMU_VARIABLES------------------
  MPU9250 mpu;
  float pitch=0;

 double MSE;
 int resolution = 770.0; // encoder resolution
 int lockTable[] = {25,40,60,80,90,110,140,160,180,200,220,240,260,280,300,320,340,360,-25,-40,-60,-70,-90,-110,-120,-140,-160,-180}; // lock table
  static int  prevAngle=0;
  float angle=0;
 int old_pitch=0;
// Quadrature encoder input pins
 const int encoderPinA = 2;
 const int encoderPinB = 3;
 const int motorDirectionPin1 = 8;
 const int motorDirectionPin2 = 9;
 const int motorSpeedPin = 10;

// Variables for storing encoder state and count
volatile static  int encoderCount = 0;
volatile int encoderStateA = LOW;
volatile int encoderStateB = LOW;
volatile int lastEncoderState = LOW;

double performance(double Kp, double Ki, double Kd, int pitch, int angle ) {
  // Set the PID gains
  myPID.SetTunings(Kp, Ki, Kd);
  
  // Initialize the PID controller

  double time_step = 0.09;
  double duration = 1;
  double target = pitch;
  double output = 0.0;
  unsigned long previousMillis = 0;
  unsigned long currentMillis = 0;
  int num_samples=100;
  double itae;
 for (int i = 0; i < num_samples; i++) {

  
  {
   cli();
    Input = angle;
    Setpoint = abs(pitch);
   myPID.Compute();
    output = Output;
    error = Setpoint - output;
    MSE  += pow(error, 2);
    sei();
  }
 }

  MSE /= num_samples;
   // Serial.print(itae);
  // Serial.print(MSE);
    //Serial.print("\n");
 //Serial.print(" the performance is = \n'");
 return MSE;

}
void tune_PID(double & Kp, double & Ki, double & Kd,int pitch , int angle ) {
  // Use coordinate ascent to tune the PID gains

  double best_performance = performance(Kp, Ki, Kd,pitch,angle);

  // Initialize the previous MSE to a very large number
      if(   num_no_improvement>30){
     delta_kp*=1.3;
    delta_ki*=1.3; 
      delta_kd*=1.3;
    num_no_improvement=0;
     }
      if(   num_no_improvement_kp>20){
     delta_kp/=2;
     num_no_improvement_kp=0;
     }
     if(   num_no_improvement_ki>20){
     delta_ki/=2;
      num_no_improvement_ki=0;
      }
      if(   num_no_improvement_kd>20){
       delta_kd/=2; 
        num_no_improvement_kd=0;
        }
    // Tune Kp
    double kp_plus = Kp + delta_kp;
    double kp_minus = Kp - delta_kp;
    double perf_kp_plus = performance(kp_plus, Ki, Kd,pitch,angle);
    double perf_kp_minus = performance(kp_minus, Ki, Kd,pitch,angle);
    if (perf_kp_plus > best_performance) {
      Kp = kp_plus;
      best_performance = perf_kp_plus;

    } else if (perf_kp_minus > best_performance) {
      Kp = kp_minus;
      best_performance = perf_kp_minus;

    } else 
     {
      num_no_improvement_kp++;
     }
    
    // Tune Ki
    double ki_plus = Ki + delta_ki;
    double ki_minus = Ki - delta_ki;
    double perf_ki_plus = performance(Kp, ki_plus, Kd,pitch,angle);
    double perf_ki_minus = performance(Kp, ki_minus, Kd,pitch,angle);
    if (perf_ki_plus > best_performance) {
      Ki = ki_plus;
      best_performance = perf_ki_plus;

    } else if (perf_ki_minus > best_performance) {
      Ki = ki_minus;
      best_performance = perf_ki_minus;

    } else {
   num_no_improvement_ki++;
       }
    
    // Tune Kd
    double kd_plus = Kd + delta_kd;
    double kd_minus = Kd - delta_kd;
    double perf_kd_plus = performance(Kp, Ki, kd_plus,pitch,angle);
    double perf_kd_minus = performance(Kp, Ki, kd_minus,pitch,angle);
    if (perf_kd_plus > best_performance) {
      Kd = kd_plus;
      best_performance = perf_kd_plus;

    } else if (perf_kd_minus > best_performance) {
      Kd = kd_minus;
      best_performance = perf_kd_minus;

    } else {
   num_no_improvement_kd++;

    }
    
    // Check if the performance has improved
 
    double itae = best_performance;
    
    if (itae < prev_itae) {
     num_no_improvement = 0;
    } else {
       num_no_improvement ++;
    }
    
    // Update the previous MSE
     prev_itae = itae;
 



Serial.print("Kp, Ki, Kd: ");
Serial.print(Kp, 2);
Serial.print(", ");
Serial.print(Ki, 2);
Serial.print(", ");
Serial.println(Kd, 2);

  myPID.SetTunings(Kp, Ki, Kd);
}
void mpu_setup(){
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
  }
    mpu.verbose(true);
    mpu.calibrateAccelGyro();

 
   // mpu.calibrateMag();

    mpu.verbose(false);

}
int get_pitch(){
  if (mpu.update()){  
 pitch=mpu.getPitch();
  // Serial.print(pitch);
 //Serial.print("\n ");
 }
 bool found2 = false;
  for (int i = 0; i < sizeof(lockTable) / sizeof(int); i++) {
    if (pitch >= lockTable[i] && pitch < lockTable[i] + 20) {
      pitch = lockTable[i];
      found2 = true;
      break;
    }
  }
  if (!found2) {
    pitch = 0;
  }
 return pitch; 
}
void motor_write(int angle,int pitch) 
{
 // Compute PID output

  // Set motor direction based on PID output sign

 

   int tolernce=abs(angle)-abs(pitch);
   // Check if desired angle is reached
  //  Serial.print(tolernce);

 //Serial.print("\n'");
   
   
   
   if (abs(tolernce) <= 10 ) 
   {
 do {
    digitalWrite(motorSpeedPin, HIGH);
    digitalWrite(motorDirectionPin2,LOW);
   digitalWrite(motorDirectionPin1, LOW);
    int x =get_pitch();
   int tolerence_1=abs(angle)-abs(x);
 }while(abs(tolerence_1)<=10);    
    
    
    }
    else if( pitch==0 || (abs(tolernce) <= 10 ) )
    {
    digitalWrite(motorSpeedPin, HIGH);
    digitalWrite(motorDirectionPin2,LOW);
    digitalWrite(motorDirectionPin1, LOW);
    }  
    else 
    {
    analogWrite(motorSpeedPin, 50);
    
    if(pitch>0) 
 {
   if(pitch>old_pitch || pitch==old_pitch)
   {
   digitalWrite(motorDirectionPin2,HIGH );
   digitalWrite(motorDirectionPin1, LOW);
   // analogWrite(motorSpeedPin, 100);
   }
   else if(pitch<old_pitch){
    digitalWrite(motorDirectionPin2,LOW );
   digitalWrite(motorDirectionPin1,HIGH);
    }
  
  }
 else if(pitch<0){
   if(pitch<old_pitch || pitch==old_pitch)
   {
   digitalWrite(motorDirectionPin2,LOW );
   digitalWrite(motorDirectionPin1, HIGH);
   // analogWrite(motorSpeedPin, 100);
   }
   else if (pitch>old_pitch){
    digitalWrite(motorDirectionPin2,HIGH );
   digitalWrite(motorDirectionPin1, LOW);
    //analogWrite(motorSpeedPin, 100);

   }
}
    }
 
 
 old_pitch=pitch;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
       Wire.begin();
   mpu_setup();
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
   attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,80);


}

void loop() 
{
  get_pitch();
  angle=(float) prevAngle *(360.0/resolution);
   //Serial.print(angle, 2);
   //Serial.print("input: ");
   bool found1 = false;
  for (int i = 0; i < sizeof(lockTable) / sizeof(int); i++) {
    if (angle >= lockTable[i] && angle < lockTable[i] + 20) {
      angle = lockTable[i];
      found1 = true;
      break;
    }
  }
  if (!found1) {
    angle = 0;
  }
  for (int i = 0; i < sizeof(lockTable) / sizeof(int); i++) {
    if (pitch >= lockTable[i] && pitch < lockTable[i] + 20) {
      pitch = lockTable[i];
      found2 = true;
      break;
    }
  }
  if (!found2) {
    pitch = 0;
  }
   if(pitch==0){
    prevAngle=0;
   
   }
    Input = angle;
   Serial.print(Input, 2);
   Serial.print("input: ");
Serial.print(", \n");
    Setpoint = abs(pitch);
  tune_PID(Kp,Ki,Kd,pitch,angle);

   myPID.Compute();
  motor_write(Output, pitch);
/*
Serial.print("output,pitch: ");
Serial.print(Output, 2);
Serial.print(", \n");
Serial.print(pitch, 2);
*/
}
void encoderISR() {
   uint8_t gray = 0;
   uint8_t state = 0;
   uint8_t prevState = 0;
   uint8_t grayTable[] = {0, 1, 3, 2, 1, 0, 2, 3, 3, 2, 0, 1, 2, 3, 1, 0};



  int rotaryA = digitalRead(encoderPinA);
  int rotaryB = digitalRead(encoderPinB);
  uint8_t binary = (rotaryA << 1) | rotaryB;
  uint8_t newGray = prevState ^ grayTable[binary];

  //gray = grayTable[state];

  switch (newGray) {
    case 1:
      prevAngle++;
      break;
    case 2:
      prevAngle--;
      break;
    case 3:
      prevAngle = prevAngle + 2;
      break;
    default:
      // Invalid Gray code value, do nothing
      break;
  }

  if(prevAngle>=770){
  prevAngle=0;
 }
  prevState = binary;
}


