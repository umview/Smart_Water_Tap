int n=0;
int tmp=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(0,record,RISING);
}
void record(){
  n++;
  //delay(10);
  //if(n>10)tmp=1;
 Serial.println(n);
}
void loop() {
  // put your main code here, to run repeatedly:
//delay(1000);
//Serial.println(n);
}
