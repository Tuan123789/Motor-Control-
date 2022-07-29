#include <Encoder.h>
#include <PID_v1.h>
#include <Math.h>

#define ENA 12 // PWM outputs to L298N H-Bridge motor driver module
const int IN1=23;
const int IN2=22;
#define ENB 9 // PWM outputs to L298N H-Bridge motor driver module
const int IN3=25;
const int IN4=24;
#define ENC 11 // PWM outputs to L298N H-Bridge motor driver module
const int IN5=26;
const int IN6=27;
#define END 10 // PWM outputs to L298N H-Bridge motor driver module
const int IN7=28;
const int IN8=29;
#define CH1 3
#define CH2 5
#define CH3 4
#define CH5 2

float A = 0;
float B = 0;
float C = 0;
double r =0.1;
double kp1 = 0.2, ki1 = 0, kd1 = 0; // modify for optimal performance
double kp2 = 0.2, ki2 = 0, kd2 = 0; // modify for optimal performance
double input1 = 0, output1 = 0, setpoint1 = 9600;
double input2 = 0, output2 = 0, setpoint2 = 9600;
double starttime;
double encoder_res = 2400; // enconder resolution 
Encoder myEnc1(20,21);
Encoder myEnc2(18,19); // xanh lá là chân số 5  

PID myPID1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT' 
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

int dem1 = 0 ;
int dem2 = 0 ;
float outputt1 = 0;
float outputt2 = 0;
float outputt3 = 0;
float outputt4 = 0;
float ga = 0;
float ga1 = 0;
// Integers to represent values from sticks and pots
float ch1Value=0;
float ch2Value=0;
float ch3Value=0;
bool ch5Value;

char buffer[4]= {'0','S','0','D'};

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
 
// modify for optimal performance
void setup() {
  //toggleSwitch.setDebounceTime(50);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(ENB, HIGH);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  digitalWrite(ENC, HIGH);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  digitalWrite(END, HIGH);
  Serial.begin(9600);
  //digitalWrite(13, HIGH);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(1);
  myPID1.SetOutputLimits(-255, 255);
  
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(1);
  myPID2.SetOutputLimits(-252, 255);
  starttime = 0; 
  input1 = myEnc1.read(); // read value of original position
  input2 = myEnc2.read(); // read value of original position
}

void loop() {
  //toggleSwitch.loop();
  //int state = toggleSwitch.getState();
  if(Serial.available()>=4){
   Serial.readBytes(buffer,4);
   if(buffer[1]== 'M') A = float(int(buffer[2])-48);
   if(buffer[1]== 'N') B = float(int(buffer[2])-48);
   //Serial.print("Toi nhan duoc: ");
   //Serial.println(buffer);
  }
  ga1 = (int(buffer[2])-48)*25;
  //Serial.println(ga1);
  if(buffer[0]=='0'){
    //Serial.println(buffer);
    Drive_Motor1(outputt1);
    Drive_Motor2(outputt2);
    Drive_Motor3(outputt3);
    Drive_Motor4(outputt4);
    ch1Value = readChannel(CH1, 0, 100, 0);
    ch2Value = readChannel(CH2, 0, 100, 0);
    ch3Value = readChannel(CH3, 0, 100, 0);
    ch5Value = readSwitch(CH5, false);
    
    ga = ((ch3Value-16)/72)*255;
    if ((ch1Value==0)||(ch2Value==0)||(ch1Value>40&&ch1Value<60)||(ch2Value>40&&ch2Value<60)||buffer[1] == 'S'){
        outputt1 = 0;
        outputt2 = 0;
        outputt3 = 0;
        outputt4 = 0;
    }
    if (ch1Value>60){
      outputt1 = ga;
      outputt2 = ga;
      if (ch5Value == 1){ 
        outputt3 = 255;
        outputt4 = 255;
      }
      else {
        outputt3 = 0;
        outputt4 = 0;
      }
    }
    if (buffer[1] == 'T'){
      outputt1 = ga1;
      outputt2 = ga1;
      if (buffer[3] == 'A'){ 
        outputt3 = 255;
        outputt4 = 255;
      }
      else if(buffer[3] == 'B'){
        outputt3 = 255;
        outputt4 = 200;
      }
      else if(buffer[3] == 'C'){
        outputt3 = 255;
        outputt4 = 170;
      }
      else{
        outputt3 = 0;
        outputt4 = 0;
      }
    }
    if ((ch1Value<40&&ch1Value>0)){
      outputt1 = -ga;
      outputt2 = -ga;
    }
    if (buffer[1] == 'L'){
      outputt1 = -ga1;
      outputt2 = -ga1;    
    }
    if(ch2Value>70){
      outputt1 = -ga;
      outputt2 = ga;
    }
    if(buffer[1] == 'R'){
      outputt1 = -ga1;
      outputt2 = ga1;
    }
    if ((ch2Value<40&&ch2Value>0)){
      outputt1 = ga;
      outputt2 = -ga;
    }
    if (buffer[1] == 'P'){
      outputt1 = ga1;
      outputt2 = -ga1;
    }
  }
  if(buffer[0] == '1'){
    double nowtime = millis();
    C = A+B/10;
    setpoint1 = 15278.87454*C;
    setpoint2 = 15278.87454*C;
    if(setpoint1 == 0) myEnc1.write(0);
    if(setpoint2 == 0) myEnc2.write(0);
    if(C != 0){
    if (buffer[3] == 'A'){ 
      outputt3 = 255;
      outputt4 = 255;
    }
    else if(buffer[3] == 'B'){
      outputt3 = 255;
      outputt4 = 200;
    }
    else if(buffer[3] == 'C'){
      outputt3 = 255;
      outputt4 = 170;
    }
    else{
      outputt3 = 0;
      outputt4 = 0;
    }
    }
    Drive_Motor3(outputt3);
    Drive_Motor4(outputt4);
        
  //Serial.println(nowtime);
    if((nowtime - starttime) >= 10) {
      //Serial.println("abc");
      //Serial.println(delta_time);
      setpoint1 += convert(0);
      setpoint2 += convert(0);
      //setpoint1 = 9600;
      //setpoint2 = 9600;  
      input1 = myEnc1.read();
      input2 = myEnc2.read();
      //Serial.println(input1);
      myPID1.Compute();
      myPID2.Compute();
      //Serial.println(output1);
      Drive_Motor1(output1);
      Drive_Motor2(output2);
      if (output1 <=50) {
        buffer[3] = 'D';
        outputt3 = 0;
        outputt4 = 0;
      }
      
      starttime = millis();
    }
  }
}
void Drive_Motor1(int out) {
  //Serial.println(out);
  if (out > 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA, out);
  }else {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA, abs(out));
  }
}

void Drive_Motor2(int out) {
  //Serial.println(out);
  if (out > 0){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(ENB, out);
  }else {
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(ENB, abs(out));
  }
}

void Drive_Motor3(int out) {
  //Serial.println(out);
    digitalWrite(IN5,LOW);
    digitalWrite(IN6,HIGH);
    analogWrite(ENC, out);
}

void Drive_Motor4(int out) {
  //Serial.println(out);
    digitalWrite(IN7,LOW);
    digitalWrite(IN8,HIGH);
    analogWrite(END, out);
}

double convert(double tocdo) {
  double y = ((60*tocdo))/((2*3.14*r)); // vòng/phút
  //Serial.println(y);
  double pulse = (y*encoder_res)/(60*100); // convert to pulse/10ms
  //Serial.println(pulse);
  return pulse;
}
