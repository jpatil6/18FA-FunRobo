#include <SharpIR.h>      // IR sensors library


//SharpIR IR1(SharpIR::GP2Y0A02YK0F, A8);
//SharpIR IR2(SharpIR::GP2Y0A02YK0F, A9);
//SharpIR IR3(SharpIR::GP2Y0A02YK0F, A10);
//SharpIR IR4(SharpIR::GP2Y0A02YK0F, A11);
//SharpIR IR5(SharpIR::GP2Y0A02YK0F, A12);
//SharpIR IR6(SharpIR::GP2Y0A02YK0F, A13);

int IRarray[6] = {};
int previousmillis;

const int sonar1 = A3;  //sets signal pin for first sonar sensor
const int sonar2 = A4;  //sets signal pin for second sonar sensor
const int sonar3 = A5;  //sets signal pin for third sonar sensor
int trigger = 12;  //sets 1 trigger pin for all 3 sensors
int sonarArray[3] = {};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigger, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readIR();
  readSonar();
      Serial.println("IRData: ");
    Serial.println(sizeof(IRarray));
      for (int i = 0; i < 6 ; i++) {
        Serial.print(IRarray[i]);
        Serial.print(", ");
  }
  Serial.println(" ");
  Serial.println("SonarData: ");
  for (int i = 0; i < 3 ; i++) {
    Serial.print(sonarArray[i]);
    Serial.print(", ");
  }

  Serial.println(" ");

  delay(500);
  //Serial.println(millis()- previousmillis);
  previousmillis = millis();
}

void readIR() {
  /* Uses the getDistance function from SharpIR library to get distances in cm*/
  IRarray[0] = averageOut(A8);
  IRarray[1] = averageOut(A9);
  IRarray[2] = averageOut(A10);
  IRarray[3] = averageOut(A11);
  IRarray[4] = averageOut(A12);
  IRarray[5] = averageOut(A13);


  //Serial.println(sizeof(IRarray));
}

void readSonar() {
  digitalWrite(trigger, HIGH);
  delay(1);
  digitalWrite(trigger, LOW);
  sonarArray[0] = analogRead(sonar1) / 2;
  sonarArray[1] = analogRead(sonar2) / 2;
  sonarArray[2] = analogRead(sonar3) / 2;
}

int averageOut( uint8_t pin) {
  int distance = 0;
  int i = 1;
  while (i < 50) {
    //int distance = sensor.getDistance(); //Calculate the distance in centimeters and store the value in a variable
    distance += int(pow(analogRead(pin), -1.02) * 13755);
    i ++;
  }
  distance = int(distance / (i - 1));
  if (distance < 20 || distance > 120){
    distance = 120;
  }
  return distance;
}
