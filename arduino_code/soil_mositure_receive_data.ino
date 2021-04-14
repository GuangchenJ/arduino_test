/*
  【Arduino】66种传感器模块系列实验（53）
  实验五十三： 土壤湿度水分传感器模块（电阻式）
  实验二，黑板四线制
*/

#define AO A0
#define DO A1

void setup()
{
  pinMode(AO, INPUT);
  pinMode(DO, INPUT);
  Serial.begin(9600);
}

void loop()
{
  Serial.print("AO=");
  Serial.print(analogRead(A0));
  Serial.print("|DO=");
  Serial.println(digitalRead(DO));
  delay(2000);
}