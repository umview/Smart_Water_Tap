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
const int EchoPin[] = {2,4,6,8,10,12}; 
const int TrigPin[] = {3,5,7,9,11,13}; 
float distance; 
int getDistance(int n){
digitalWrite(TrigPin[n], LOW); //低高低电平发一个短时间脉冲去TrigPin 
delayMicroseconds(2); 
digitalWrite(TrigPin[n], HIGH); 
delayMicroseconds(10); 
digitalWrite(TrigPin[n], LOW); 
distance=pulseIn(EchoPin[n], HIGH) / 58.0; //将回波时间换算成cm 
//distance = (int(distance * 100.0)) / 100.0; //保留两位小数  
if(distance>1000)distance=1000;
return(distance);
}
void setup() 
{ 
Serial.begin(115200); 
for(int i=0;i<=5;i++){
pinMode(TrigPin[i], OUTPUT); 
pinMode(EchoPin[i], INPUT); 
}
} 

void loop() 
{ 
    SDS_OutData[0]=getDistance(0);
    SDS_OutData[1]=getDistance(1);
    SDS_OutData[2]=getDistance(2);
    SDS_OutData[3]=getDistance(3);
    SDS_OutPut_Data(SDS_OutData);
    delay(50);
   
  } 
  /*
  if(pos>posmax-1){
    pos=posmin;
    myservo.write(pos);
    delay(500);
  }*/


