//Grupo6_Kalik_Mofsovich_Chames_Brubiski
int MQ2 = A5;

void setup() {
  Serial.begin(115200);

  pinMode(MQ2, INPUT);

}

void loop() {
  int lecturaSensor = analogRead(MQ2);
  Serial.print("Lectura: ");
  Serial.println(lecturaSensor);

}
