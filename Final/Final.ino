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
/*****************************************************************************************/
/*************************************************************/
int cnt=0;
/*************************************************************/
int8_t TrigPin=4;
int8_t EchoPin=5;
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
#define getPos 
void setup() {
  // put your setup code here, to run once:
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(6,OUTPUT);
digitalWrite(6,HIGH);
pinMode(9,INPUT);
pinMode(TrigPin, OUTPUT); 
pinMode(EchoPin, INPUT); 
attachInterrupt(0,record,RISING);
Serial.begin(115200);
  //while(!digitalRead(9))down();
  //cnt=0;
}
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
void loop() {
  float t=0;
  // put your main code here, to run repeatedly:
/*
  digitalWrite(4,HIGH);
  digitalWrite(10,HIGH);
  delay(500);
  Serial.println("OK");
  digitalWrite(4,LOW);
  digitalWrite(10,LOW);
  delay(500);
 */
 //Serial.println(getDistance());
 //SDS_OutData[0]=getDistance();
 SDS_OutData[1]=cnt;
 //SDS_OutPut_Data(SDS_OutData);
delay(500);
Serial.print(digitalRead(9));
Serial.print(" ");
Serial.print(getDistance());
Serial.print(" ");
Serial.println(cnt);
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
else stop();
}
}
