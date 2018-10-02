#include<SoftwareSerial.h>
SoftwareSerial T(10, 11); // RX, TX
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
void setup() {
  // initialize serial communications at 9600 bps:
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  Serial.begin(115200);
  T.begin(115200);
}
int value[6]={0};
void getAveAD(){
  byte i=0;
  for(i=0;i<6;i++)value[i]=0;
  for(i=0;i<10;i++){
    value[0]+=analogRead(0);
    value[1]+=analogRead(1);
    value[2]+=analogRead(2);
    value[3]+=analogRead(3);
    value[4]+=analogRead(4);
    value[5]+=analogRead(5);
    delay(5);
  }
  for(i=0;i<6;i++)value[i]/=10;
//  return value/10;
}
uint16_t AD[6],ad[6];
uint16_t ADStandard[6]={0};
void getADStandard(uint16_t t[]){
  uint16_t i=0;
  long tmp[6]={0};
  for(i=0;i<50;i++){
    tmp[0]+=analogRead(0);
    tmp[1]+=analogRead(1);
    tmp[2]+=analogRead(2);
    tmp[3]+=analogRead(3);
    tmp[4]+=analogRead(4);
    tmp[5]+=analogRead(5);
    delay(5);
  }
  t[0]=(uint16_t)(tmp[0]/50.0);
  t[1]=(uint16_t)(tmp[1]/50.0);
  t[2]=(uint16_t)(tmp[2]/50.0);
  t[3]=(uint16_t)(tmp[3]/50.0);
  t[4]=(uint16_t)(tmp[4]/50.0);
  t[5]=(uint16_t)(tmp[5]/50.0);
}
int H=0;
void getAD(){
   getAveAD();
  if((int)(ADStandard[5]-value[5])>10){
    H=60;
  }else if((int)(ADStandard[4]-value[4])>8){
    H=50+((int)(ADStandard[5]-value[5]))/2;
  }else if((int)(ADStandard[3]-value[3])>8){
    H=40+((int)(ADStandard[4]-value[4]))/2;
  }else if((int)(ADStandard[2]-value[2])>8){
    H=30+((int)(ADStandard[3]-value[3]))/2;
  }else if((int)(ADStandard[1]-value[1])>8){
    H=20+((int)(ADStandard[2]-value[2]))/2;
  }else if((int)(ADStandard[0]-value[0])>8){
    H=10+((int)(ADStandard[1]-value[1]))/2;
  }else H=0; 
}
void loop(){
  delay(1000);
  getADStandard(ADStandard);
  for(;;){
  //getAD();
  H++;
  H=H>250?0:H;
  SDS_OutData[0]=H;
  SDS_OutData[1]=ADStandard[5]-value[5];
  SDS_OutData[2]=ADStandard[4]-value[4];
  SDS_OutData[3]=ADStandard[3]-value[3];
  //Serial.write((int)(H));
  Serial.println(H);
 //SDS_OutPut_Data(SDS_OutData);
  
  digitalWrite(13, HIGH);
  //Serial.write(H);
  //T.write((int)(H));
  Serial.println(H);
  digitalWrite(13, LOW);
  delay(200);
  }
  //delay(5);
}
