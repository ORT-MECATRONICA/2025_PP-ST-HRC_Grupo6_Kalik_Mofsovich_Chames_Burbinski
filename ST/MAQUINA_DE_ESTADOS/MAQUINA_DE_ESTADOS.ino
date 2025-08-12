//Grupo1_Kalik_Mofsovich_Chames_Burbinski

//Librerias
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  //Pantalla
#include <SPI.h>
#include <Adafruit_BMP280.h>  //Sensor de temperatura y presion
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>  //Libreria para conectar con telegram
#include <ArduinoJson.h>           //Libreria necesaria para el bot de telegram
#include <Preferences.h>           //EEPROM
#include <ESP32Time.h>
#include "time.h"

//Doble Loop
TaskHandle_t Task1;  //Loop1
TaskHandle_t Task2;  // Loop2

//Wifi
const char* ssid = "ORT-IoT";
const char* password = "NuevaIOT$25";

// Initialize Telegram BOT
#define BOTtoken "7525131885:AAFPvWp5yNZh9hGeqOli2F-FkV-qupSYYzs"  // El bor roken para que el ESP32 pueda interactura con el ESP32
#define CHAT_ID "7982476800"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

//Ajustes de la pantalla LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);

//bmp280 Sensor de temperatura y presion
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)
Adafruit_BMP280 bmp;
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

//Eeprom
Preferences preferences;

//Ajustes del Tiempo GMT
ESP32Time rtc;
int GMT;
int currentGMT = -3;

//Variables de millis()
unsigned long TiempoUltimoCambio = 0;
unsigned long TiempoUltimoCambio2 = 0;
long Intervalo;
unsigned long lastTimeBotRan = 0;
const int botRequestDelay = 2000;
unsigned long TiempoAhora;

//Estados de la maquina de estados y mas
#define PANTALLA1 1
#define ESTADO_CONFIRMACION1 2
#define ESTADO_CONFIRMACION2 3
#define PANTALLA_AJUSTE 4
#define SUBIR 5
#define BAJAR 6
#define AJUSTE_GMT 7
#define AJUSTE_UMBRAL 8
#define AJUSTE_MQTT 9
void MAQUINA_DE_ESTADOS();

#define PULSADO 0
#define NO_PULSADO 1

#define BOTON1 34
#define BOTON2 35
#define BOTON3 36
#define BOTON4 37
#define BOTON5 38
#define LED1 25
#define LED2 26
#define LED3 27

//Variables generales
int lectura1;
int lectura2;
int lectura3;
int lectura4;
int lectura5;
int estado = 1;
int estadoAnterior;
int VALOR_UMBRAL = 28;
int flag_umbral = 0;
float t;

void setup() {
  Serial.begin(115200);

  //BMP280
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Pinmodes
  pinMode(BOTON1, INPUT);
  pinMode(BOTON2, INPUT);
  pinMode(BOTON3, INPUT);
  pinMode(BOTON4, INPUT);
  pinMode(BOTON5, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  xTaskCreatePinnedToCore(
    Task1code,  //La funcion donde se ejecuta el codigo del loop
    "Task1",    //Un nombre
    10000,      //El tamaño de memoria que se le asigna a la tarea en bytes o palabras
    NULL,       //No pasa ningun dato
    1,          //Prioridad de la tarea
    &Task1,     //La variable que se define al inicio, permite pausar, elminar, reanudar, o cambiar la prioridad
    0);         //El nucleo que va a utilizar
  delay(500);

  xTaskCreatePinnedToCore(
    Task2code,  //La funcion donde se ejecuta el codigo del loop
    "Task2",    //Un nombre
    10000,      //El tamaño de memoria que se le asigna a la tarea en bytes o palabras
    NULL,       //No pasa ningun dato
    1,          //Prioridad de la tarea
    &Task2,     //La variable que se define al inicio, permite pausar, elminar, reanudar, o cambiar la prioridad
    1);         //El nucleo que va a utilizar
  delay(500);

  //Display
  lcd.init();
  lcd.backlight();
  lcd.print("Hola Mundo");

  //Eeprom
  preferences.begin("Eeprom", false);                      //False significa modo leer y escribir, en True seria solo leer //El "Eeprom" es el nombre del esapcio que se crea
  VALOR_UMBRAL = preferences.getUInt("VALOR_UMBRAL", 28);  //Obtiene el valor //get Uint porque es Usigned int, y el 28 es un valor predeterminado si no hay un "Valor UMBRAL" devuelve 28
  currentGMT = preferences.getInt("currentGMT", -3);
  Intervalo = preferences.getLong("Intervalo", 5000);
  
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setInsecure();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  bot.sendMessage(CHAT_ID, "Bot started up", "");

  syncTimeWithGMT(currentGMT);

  delay(2000);
}

//Loop 1
void Task1code(void* pvParameters) {
  for (;;) {  //El for permite que se quede en loop
    //Serial.println(CONTADOR);
    MAQUINA_DE_ESTADOS();
  }
}

//Loop 2 (bot de telegram)
void Task2code(void* pvParameters) {
  for (;;) {
    if (millis() - lastTimeBotRan >= botRequestDelay) {                    //delay sin bloqueo de 2 segundos
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);  //Solicita el mensaje mas reciente

      while (numNewMessages) {                       //Si hay mensajes nuevos
        for (int i = 0; i < numNewMessages; i++) {   //Por cada mensaje
          String text = bot.messages[i].text;        //Recibe el mensaje
          String chat_id = bot.messages[i].chat_id;  //Accede al chat

          if (text == "/temperatura") {
            if (isnan(t)) {
              bot.sendMessage(chat_id, "⚠️ Error al leer la temperatura", "");
            } else {
              String temp_msg = "🌡️ La temperatura actual es: " + String(t) + " °C";
              bot.sendMessage(chat_id, temp_msg, "");
            }
          } else {
            bot.sendMessage(chat_id, "Comando no reconocido. Usa /temperatura", "");
          }
        }
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      lastTimeBotRan = millis();
    }

    //Si la temperatura supera el umbral
    if (flag_umbral == 1) {
      digitalWrite(LED1, HIGH);
      bot.sendMessage(CHAT_ID, "La temperatura supero el umbral", "");

      flag_umbral = 2;  // este flag es para que no se repita el mensaje todo el rato
    }
  }

  //vTaskDelay(100 / portTICK_PERIOD_MS); // Pausa de 100 ms Esto reduce la carga del procesador y mejora la estabilidad.
}

void loop() {}

void MAQUINA_DE_ESTADOS() {
  lectura1 = digitalRead(BOTON1);
  lectura2 = digitalRead(BOTON2);
  lectura3 = digitalRead(BOTON3);
  lectura4 = digitalRead(BOTON4);
  lectura5 = digitalRead(BOTON5);

  t = bmp.readTemperature();

  TiempoAhora = millis();
  switch (estado) {
    case PANTALLA1:  //Se ven los valores de los sensores
      estadoAnterior = estado;


      break;

    case ESTADO_CONFIRMACION1:  //Mini-antirebote

      break;

    case PANTALLA_AJUSTE:  //Se ven los valores de los sensores
      estadoAnterior = estado;
      break;

    case ESTADO_CONFIRMACION2:  //Mini-antirebote

      break;

    case AJUSTE_GMT:
      estadoAnterior = estado;
      break;

    case AJUSTE_UMBRAL:
      estadoAnterior = estado;
      break;

    case AJUSTE_MQTT:
      estadoAnterior = estado;
      break;

    case SUBIR:

      break;

    case BAJAR:

      break;
  }
}
//preferences.putUInt("VALOR_UMBRAL", VALOR_UMBRAL);  //Guarda el valor Umbral en una Key llamada "VALOR UMBRAL"

void syncTimeWithGMT(int gmtOffsetHours) {
  long gmtOffset_sec = gmtOffsetHours * 3600;
  configTime(gmtOffset_sec, 0, "pool.ntp.org");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);  // Ajusta la hora en el ESP32Time
    Serial.println("Hora sincronizada desde NTP");
  } else {
    Serial.println("Fallo al obtener la hora desde NTP");
  }
}