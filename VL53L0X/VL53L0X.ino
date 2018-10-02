float SDS_OutData[4];
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void SDS_OutPut_Data(float S_Out[])
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
 // float SDS_OutData[4];
  /*for(i=0;i<4;i++) {
  SDS_OutData[i]=S_Out[i];
  }*/
  for(i=0;i<4;i++)
   {

    temp[i]  = (int)SDS_OutData[i];
    temp1[i] = (unsigned int)temp[i];

   }

  for(i=0;i<4;i++)
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }

  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;

  //SDS_UART_Init();
  for(i=0;i<10;i++)
  {

    Serial.write(databuf[i]);
  }
}
/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.
//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
//#define HIGH_SPEED
#define HIGH_ACCURACY

int8_t TrigPin=4;
int8_t EchoPin=5;
void setup()
{
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(6,OUTPUT);
digitalWrite(6,HIGH);
pinMode(9,INPUT);
pinMode(TrigPin, OUTPUT); 
pinMode(EchoPin, INPUT); 
attachInterrupt(0,record,RISING);
Serial.begin(115200);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(33000);
#endif
}
int getDistance(){
  float distance;
digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin 
delayMicroseconds(2); 
digitalWrite(TrigPin, HIGH); 
delayMicroseconds(10); 
digitalWrite(TrigPin, LOW); 
distance=pulseIn(EchoPin, HIGH,5000) / 58.0; //将回波时间换算成cm 
//distance = (int(distance * 100.0)) / 100.0; //保留两位小数  
if(distance>1000)distance=1000;
return((int)distance);
}
int cnt=50;
void record(){
  cnt++;
}
void up(){
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);

}
void down(){
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
}
void stop(){
    digitalWrite(6,HIGH);
}
int buf[13];
int LiquidHigh=0;
int CupHigh=0;

void loop()
{

  int t;
  while(1){
    Serial.print("Waiting for CMD");
    Serial.println(getDistance());
    delay(10);
    if(Serial.available()){
    byte cmd=Serial.read();
    if(cmd=='1')up();
    else if(cmd=='2')down();
    else if(cmd=='c')cnt=0;
    else if(cmd=='3'){
    while(!digitalRead(9))down();
    stop();
    }
    else if (cmd=='s')stop();
    else if(cmd=='b')break;
   }
  }
    while(!digitalRead(9))down();
    stop();
    Serial.println("Initing System OK!");
   
    delay(1000);
    t=0;
    cnt=50;
    digitalWrite(TrigPin,1);
    while(digitalRead(EchoPin)==0){
    Serial.println("Waitting!");
      up();
      //delay(2);
    }
        digitalWrite(TrigPin,0);
    CupHigh=cnt-15;
    //up();
    delay(1000);
    stop();
    while(1){
      
      //Serial.print(getDistance());Serial.print(" ");
      if(Serial.available()){
        byte cmd=Serial.read();
        if(cmd=='b')break;
      }
      byte i=0;
      for(i=0;i<11;i++)buf[i]=buf[i+1];
      buf[11]=sensor.readRangeSingleMillimeters();
      buf[12]=0;
      for(i=0;i<12;i++){
        buf[12]+=buf[i];
      }
      buf[12]/=12.0;
      LiquidHigh=250-buf[12];
      Serial.println(LiquidHigh);
      Serial.println("Waiting for Water!");
      delay(500);
    }
    while(!(CupHigh-LiquidHigh<17)){
      if(Serial.available()){
        byte cmd=Serial.read();
        if(cmd=='b')break;
      }
      byte i=0;
      for(i=0;i<11;i++)buf[i]=buf[i+1];
      buf[11]=sensor.readRangeSingleMillimeters();
      buf[12]=0;
      for(i=0;i<12;i++){
        buf[12]+=buf[i];
      }
      buf[12]/=12.0;
      LiquidHigh=253-buf[12];
      Serial.println(LiquidHigh);
      Serial.println(CupHigh-LiquidHigh);
    }
    while(1){
      if(Serial.available()){
        byte cmd=Serial.read();
        if(cmd=='b')break;
      }
      Serial.println("OK!");
      Serial.println(sensor.readRangeSingleMillimeters());
      delay(500);
      
    }
  /*
    if(Serial.available()){
 byte cmd=Serial.read();
if(cmd=='1')up();
else if(cmd=='2')down();
else if(cmd=='c')cnt=0;
else if(cmd=='3'){
  while(!digitalRead(9))down();
}
else if(cmd=='4'){
  while(!(cnt>=40))up();
}
else if(cmd=='5')while(!(cnt>=50))up();
else if(cmd=='6')while(!(cnt>=60))up();
else if(cmd=='7')while(!(t>=6)){t=getDistance();up();delay(10);Serial.print(cnt+50);Serial.print(" ");Serial.println(t);}
else if(cmd=='s')stop();
}*/
/*
  SDS_OutData[1]=(int16_t)buf[12];
  SDS_OutData[0]=(int16_t)buf[11];//sensor.readRangeSingleMillimeters();
  SDS_OutData[2]=getDistance();
  SDS_OutData[3]=cnt;
 // SDS_OutData[2]=SDS_OutData[1]*0.99+SDS_OutData[0]*0.01;
  //if (sensor.timeoutOccurred()) { SDS_OutData[0]=0; }
  SDS_OutPut_Data(SDS_OutData);
  */
  //delay(5);
  //Serial.println();
}
