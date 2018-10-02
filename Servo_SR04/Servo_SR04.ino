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
#include <Servo.h> 
#include <math.h>
Servo myservo;
const int TrigPin = 11; 
const int EchoPin = 10; 
int pos = 0;
float distance; 
void getDistance(){
digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin 
delayMicroseconds(2); 
digitalWrite(TrigPin, HIGH); 
delayMicroseconds(10); 
digitalWrite(TrigPin, LOW); 
distance = pulseIn(EchoPin, HIGH) / 58.0; //将回波时间换算成cm 
distance = (int(distance * 100.0)) / 100.0; //保留两位小数  
}
#define posmin 5
#define posmax 80
void setup() 
{ 
myservo.attach(5);  // attaches the servo on pin 9 to the servo object 
Serial.begin(115200); 
pinMode(TrigPin, OUTPUT); 
pinMode(EchoPin, INPUT); 
} 
int x,y;
void loop() 
{ 

  for(pos =posmin; pos <= posmax; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(50);                       // waits 15ms for the servo to reach the position 
    getDistance();
    //Serial.print(pos);
    //Serial.print(" ");
    //Serial.println(distance);
    distance=distance>50?50:distance;
    distance*=10;
    y=(int)(distance*sin((pos-posmin)*(PI/180.0)))*1;
    x=(int)(distance*cos((pos-posmin)*(PI/180.0)))*1;
    SDS_OutData[0]=pos;
    SDS_OutData[1]=y;
    SDS_OutData[2]=(int)(distance);
    SDS_OutData[3]=x;
    SDS_OutPut_Data(SDS_OutData);
  } 
  /*
  if(pos>posmax-1){
    pos=posmin;
    myservo.write(pos);
    delay(500);
  }*/


}

