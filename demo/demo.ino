#include<SoftwareSerial.h>
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
/***************vl53l0x***********/
#include <Wire.h>
/**********************
SR04 TrigPin 12
     EchoPin 13
VL53L0X SDA A4
        SCL A5
*************************/
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address 0x29

byte gbuf[16];

/*****getDistance**********/

uint16_t getVL53L0X() {
	/*
  byte val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
  //Serial.print("Revision ID: "); Serial.println(val1);

  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  //Serial.print("Device ID: "); Serial.println(val1);

  val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
  //Serial.print("PRE_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1); 
  //Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));

  val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
  //Serial.print("FINAL_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1);
  //Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));
*/
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt = 0;
  while (cnt < 100) { // 1 second waiting time max
    //delay(1);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
//  if (val & 0x01) Serial.println("ready"); else Serial.println("not ready");

  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

  //Serial.print("ambient count: "); Serial.println(acnt);
  //Serial.print("signal count: ");  Serial.println(scnt);
  //Serial.print("distance ");  
  //if(dist>2500)dist=0;     
  //Serial.println(dist);
  //Serial.print("status: ");        Serial.println(DeviceRangeStatusInternal);
  return dist;
}

uint16_t bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}

uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
  // write data word at address and register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void write_word_data_at(byte reg, uint16_t data) {
  // write data word at address and register
  byte b0 = (data &0xFF);
  byte b1 = ((data >> 8) && 0xFF);
    
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}

byte read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

byte read_byte_data_at(byte reg) {
  //write_byte_data((byte)0x00);
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

uint16_t read_word_data_at(byte reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf); 
}

void read_block_data_at(byte reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i=0; i<sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}


uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}
#define TrigPin 12
#define EchoPin 13
float getSR04(){
     // 产生一个10us的高脉冲去触发TrigPin 
        digitalWrite(TrigPin, LOW); 
        delayMicroseconds(2); 
        digitalWrite(TrigPin, HIGH); 
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW); 
    // 检测脉冲宽度，并计算出距离
        return pulseIn(EchoPin, HIGH) / 58.00; 
}
/***********************main*********************/





#define ABS(a) (a>0?a:-a)

#define N 3
uint16_t filter()
{
   uint16_t value_buf[N];
   byte count,i,j,temp;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = requestData();
      delay(5);
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;j++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   return value_buf[(N-1)/2];
}

int data,datalast;
uint16_t tmp[8];
uint16_t Dis=0,DisLast=0;
uint16_t DisBuff[10];
uint16_t requestData(){
  uint16_t tmp=20;
  while(tmp==20)tmp=getVL53L0X();
  return tmp;
}
int disCup=0,disWater=0;
/*****************/
SoftwareSerial R(10, 11); // RX, TX
/*******/
/*******/
void setup(){
  R.begin(115200);
  Serial.begin(115200);
  Wire.begin();        // join i2c bus (address optional for master)
  pinMode(12, OUTPUT); 
  pinMode(13, INPUT); 
  Serial.println("VLX53LOX test started.");
  //attachInterrupt(0, getH, RISING);
}
int disCupLast=0,disWaterLast=0;
void loop(){
  disCupLast=disCup;
  disWaterLast=disWater;
 disWater=int(requestData());
 if(R.available()>0){
   disCup=int(R.read());
    if(disCup>100)disCup=disCupLast;
 }
 delay(20);
    SDS_OutData[0]=disCup;
    SDS_OutData[1]=disWater;
    SDS_OutPut_Data(SDS_OutData);

}
