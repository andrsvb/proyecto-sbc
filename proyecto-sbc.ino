#include <WiFi.h>                         // control WiFi para ESP32
#include <ThingsBoard.h>                  // ThingsBoard SDK
#include <ArduinoJson.h>                  //
#include <PubSubClient.h>                 //
#include <ArduinoOTA.h>                   // libreria para OTA con Arduino
#include <string>

// Configuracion del wifi al que se conecta la ESP32
#define WIFI_AP_NAME        "poco"
#define WIFI_PASSWORD       "pikachu7"

// Configuracion de ThingsBoard
#define TOKEN               "AT_sbc2020_g2"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

// Baudios del puerto serie
#define SERIAL_DEBUG_BAUD    115200

// Configuracion del cliente y la instancia de Thingsboard
WiFiClient espClient;
PubSubClient client(espClient);
ThingsBoard tb(espClient);

// the Wifi radio's status
// int status = WL_IDLE_STATUS;


// ---------------------------------------------------------------- Declaracion de librerias y variables para los sensores

// ------------------------ Configuracion del sensor de humedad y temperatura
#include "DHT.h"

#define DHT_PIN 32                        // pin de la ESP al que esta conectado el sensor de humedad y temperatura
#define DHTTYPE DHT22                     // version del sensor: DHT 22  (AM2302)

DHT dht(DHT_PIN, DHTTYPE);

#if defined(ARDUINO_ARCH_AVR)
#define SERIAL  Serial

#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
#define SERIAL  SerialUSB
#else
#define SERIAL  Serial
#endif


// ------------------------ Configuracion del sensor de luz
#define LIGHT_PIN 2                         // pin de la ESP al que esta conectado el sensor de luz


// ------------------------ Configuracion del sensor de calidad del aire
#include "Air_Quality_Sensor.h"

#define AIR_PIN 33                          // pin de la ESP al que esta conectado el sensor de calidad del aire
AirQualitySensor sensor(AIR_PIN);           // configuracion del pin del sensor de calidad del aire


// ------------------------ Variables para guardar los datos de los sensores
float temperatura = 0;
float humedad = 0;
int luzV = 0;
String luzS = "";
int calidadV = 0;
String calidadS = "";


// ----------------------------- Configuracion de la tira de leds
const int ledPin = 4;                       // pin de la ESP al que esta conectada la tira de leds


// ---------------------------------------------------------------- Configuracion inicial de la placa, ejecuta al empezar
void setup() {
    // Configuracion de baudios para la comunicacion con el monitor serie
    SERIAL.begin(SERIAL_DEBUG_BAUD);

    // Configuracion de los pines de la placa
    pinMode(ledPin, OUTPUT);             
    pinMode(AIR_PIN, INPUT);             
    pinMode(LIGHT_PIN, INPUT);         
    pinMode(DHT_PIN, INPUT);            
    
    // Conexion a la wifi y a ThingsBoard
    connectWiFi();
    client.setServer( THINGSBOARD_SERVER, 1883 );

    // Configuracion inicial de los sensores
    Serial.println("Waiting for air quality sensor to init...");
    delay(20000);
    if (sensor.init()) {
        Serial.println("Sensor ready.");
    } else {
        Serial.println("Sensor ERROR!");
    }
    Wire.begin();
    dht.begin();

    // Configuracion inicial de OTA
    setupOTA();
}

// ---------------------------------------------------------------- Codigo de ejecucion de la placa, se ejecuta em bucle
void loop() {
    // Reconectar al wifi de ser necesario
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    // Manejar las actualizaciones OTA
    ArduinoOTA.handle();
    // Leer los datos de los sensores
    actualizarDatos();
    // Enviar los datos a ThingsBoard
    enviarDatos();
    // Actualizar el estado del actuador (tira de leds)
    led();
    // Para dejar suficiente tiempo entre actualizaciones
    delay(2000);
}

// ---------------------------------------------------------------- Codigo de configuracion inicial para actualizaciones OTA
void setupOTA () {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

// ---------------------------------------------------------------- Funcion para enviar los datos a ThingsBoard
void enviarDatos()
{
    // Conectarse a Thingsboard de no estar conectado
    if (!tb.connected()) {
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
        Serial.println("Fallo al conectar");
        return;
      }else{
        Serial.println("Conexion a ThingsBoard realizada con exito");
      }
    }
   Serial.println("\nSending data...");
   tb.sendTelemetryFloat("Temperatura", temperatura);
   tb.sendTelemetryFloat("Humedad", humedad);
   tb.sendTelemetryInt("Luz", luzV);
   tb.sendTelemetryInt("Calidad aire", calidadV);
   tb.loop();
}

// ---------------------------------------------------------------- Codigo de configuracion inicial para actualizaciones OTA
void led()
{
  if(!luzV) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
}

// ---------------------------------------------------------------- Codigo de configuracion inicial para actualizaciones OTA
void actualizarDatos() {
// ------------------------ sensor de humedad y temperatura
    float temp_hum_val[2] = {0};
    // El sensor tarda 250 milisegundos en leer los valores de temperatura o humedad
    // Los datos proporcionados por el sensor pueden ser de hasta 2 segundos antes, es un sensor muy lento
    
    if(!dht.readTempAndHumidity(temp_hum_val)){
        humedad = temp_hum_val[0];
        temperatura = temp_hum_val[1];
        SERIAL.print("\nHumedad: ");
        SERIAL.print(humedad);
        SERIAL.print("%, Temperatura: ");
        SERIAL.print(temperatura);
        SERIAL.print("ºC");
    }
    else{
        SERIAL.print("\nFailed to get temperature and humidity value.");
    }
    
// ------------------------ sensor de luz
    luzV = digitalRead(LIGHT_PIN);
    if (luzV == 0) {
        luzS = (" - Sin luz");
    } else {
        luzS = (" - Con luz");
    }
    SERIAL.print("\nSensor luminico: ");
    SERIAL.print(luzV);
    SERIAL.print(luzS);

// ------------------------ sensor de calidad del aire
    int quality = sensor.slope();

    calidadV = sensor.getValue();

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        calidadS = " High pollution! Force signal active.";
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        calidadS = " High pollution!";
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        calidadS = " Low pollution!";
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        calidadS = " Fresh air.";
    }
    SERIAL.print("\nSensor calidad del aire: ");
    SERIAL.print(calidadV);
    SERIAL.print(calidadS);

}

// ---------------------------------------------------------------- Funcion para conectarse al wifi
void connectWiFi()
{
  Serial.print("Connecting to AP (");
  Serial.print(WIFI_AP_NAME);
  Serial.print(") ...");
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  delay(4000);                                // Espera para comprobar la conexion
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    Serial.print(".");
    delay(4000);                              // Espera para comprobar la conexion
  }
  Serial.println(" Connected to AP");
}
