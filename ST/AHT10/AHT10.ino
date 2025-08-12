//Grupo6_Kalik_Mofsovich_Chames_Brubiski
#include <Wire.h>
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando sensor AHT10...");

  if (!aht.begin()) {
    Serial.println("No se encontró el AHT10. Revisa las conexiones.");
    while (1) {
      delay(10);
    }
  }

  Serial.println("Sensor AHT10 encontrado.");
}

void loop() {
  sensors_event_t humidity, temp;

  aht.getEvent(&humidity, &temp); // Lee ambos en una sola llamada

  Serial.print("Temperatura: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humedad: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.println("-------------------");
  delay(1000);
}
