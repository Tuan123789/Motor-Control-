
//khai báo thư viện ezButton.h để sử dụng button
#include <ezButton.h>
#include <SoftwareSerial.h>

//khai báo chân số 7 là chân điều khiển button trên arduino
ezButton toggleSwitch(7);

#define TX_PIN      2
#define RX_PIN      3
SoftwareSerial bluetooth(RX_PIN, TX_PIN);
char buffer[5]= {};
String ch;
char SWITCH[4] = {'0','S','0','D'};
char check ='0';
void setup()
{
 Serial.begin(9600);
 bluetooth.begin(9600);
 toggleSwitch.setDebounceTime(50);
 //Serial.write(SWITCH);
}
void loop() {
  toggleSwitch.loop();
  if (toggleSwitch.isPressed()){
    SWITCH[0] = '1';
    Serial.write(SWITCH);
  }
  if (toggleSwitch.isReleased()){
    SWITCH[0] = '0';
    Serial.write(SWITCH);
    //Serial.print(SWITCH);
  }
  if(bluetooth.available())
  {
    check = bluetooth.peek();
    if( int(check) <=64){
      SWITCH[2]=bluetooth.read();
//      Serial.println(buffer);
//      //SWITCH[2] = float(bluetooth.parseFloat());
//      //Serial.println(float(SWITCH[2]));
//      //Serial.println(float(SWITCH[2])-48);
    }
    else if ((int(check) >=64)&&(int(check) <=68)) SWITCH[3]=bluetooth.read();
    else SWITCH[1] = bluetooth.read();
    Serial.write(SWITCH);
    //Serial.println(SWITCH);
//     for (int i=0; i<4; i++) {
//    buffer[i] = bluetooth.read();}
//    Serial.println(buffer);
  }
   //Serial.print(ch1);
}
