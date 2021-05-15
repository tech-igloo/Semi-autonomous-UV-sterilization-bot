#define ENCODER_A 4
#define ENCODER_B 5

int del = 1;
void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, OUTPUT);
  pinMode(ENCODER_B, OUTPUT);
}

void loop() {
  if (Serial.available() > 0){  
    int x = Serial.parseInt();
    if(x>0)
      del = x;
      Serial.println(del);
  }
  //Enter 1 if want to stop the feedback
  if (del<2){
    digitalWrite(ENCODER_A, LOW);
    digitalWrite(ENCODER_B, LOW);
  }
  else{
    digitalWrite(ENCODER_A, HIGH);
    digitalWrite(ENCODER_B, HIGH);
    delay(del);  //Time period of sq wave at default del is 18ms
    digitalWrite(ENCODER_A, LOW);
    digitalWrite(ENCODER_B, LOW);
    delay(del);
  }
  //Serial.println(del);
}
