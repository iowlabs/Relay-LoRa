#include <Arduino.h>
#include <ArduinoJson.h>
#include <LoRa.h>
#include <WiFiManager.h>
#include "esp_timer.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "WiFiClientSecure.h"

 //Estos pines fueron utilizados en la prueba de concepto realizada en placa perforada con los componentes modulares, estan sujetos a cambios en la realización de una placa
#define RFM_CS       5
#define RFM_RST      17
#define RFM_DIO0     4
#define Rele     32
#define Wifistatus     33
#define StatusSW     34

int releState = 0;
bool wifiConnect = false;
bool wifiPreviouslyConnected = false;
int ledstate = 0;

const char * rootCACertificate = \
	"-----BEGIN CERTIFICATE-----\n" \
	"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
	"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
	"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
	"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
	"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
	"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
	"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
	"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
	"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
	"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
	"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
	"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
	"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
	"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
	"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
	"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
	"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
	"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
	"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
	"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
	"-----END CERTIFICATE-----\n";

WiFiClientSecure client;

#define URL_fw_Bin "https://raw.githubusercontent.com/Iowlabs/Relay-LoRa/main/firmware/OTAupdate/firmware.bin"

void EstadoWIFI() {
  // Si el pin StatusSW está en HIGH
  if (digitalRead(StatusSW) == HIGH) {
    wifiConnect = true; // Marcar que el Wi-Fi está conectado
    digitalWrite(Wifistatus, HIGH);
  } else {
    wifiConnect = false; // Marcar que el Wi-Fi está desconectado
    digitalWrite(Wifistatus, LOW);
  }
}

void LedWifi(){
  static unsigned long previousMillis = 0; // Tiempo anterior de ejecución de la función
  const unsigned long interval = 800; // Intervalo de tiempo para cambiar el estado del LED

  unsigned long currentMillis = millis(); // Tiempo actual
  if (currentMillis - previousMillis >= interval) { // Si ha pasado el intervalo de tiempo
    previousMillis = currentMillis; // Actualizamos el tiempo anterior

    if (wifiConnect == 1 && wifiPreviouslyConnected) { // Si el Wi-Fi está conectado y estaba conectado previamente
      ledstate = !ledstate; // Cambiamos el estado del LED
      digitalWrite(Wifistatus, ledstate); // Encendemos o apagamos el LED
    } else { // Si el Wi-Fi no está conectado o no estaba conectado previamente
      digitalWrite(Wifistatus, LOW); // Apagamos el LED
    }
  }
}

void ReceptorLoRa(){
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    StaticJsonDocument<128> doc_rx;
    while (LoRa.available()) {
      String receivedMessage = LoRa.readString();
      Serial.print("Mensaje recibido: ");
      Serial.println(receivedMessage);

      DeserializationError error = deserializeJson(doc_rx, receivedMessage);  //Analiza lo recibido como un JSON para encontrar la posición de RELE
                                                                              // esto es necesario en caso de recibir multiples datos, puede optimizarse de ser pocos
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
      }
      if (doc_rx.containsKey("R000")) {
        releState = doc_rx["R000"];//Busca "Rele" para saber su estado
        Serial.print("Estado del rele: ");
        Serial.println(releState);
      } else {
        Serial.println("El JSON no contiene la clave 'Rele'");
      }
    }
  }
}

void ManejoWifi(){
  if (wifiConnect == 1 && !wifiPreviouslyConnected) {
    // Creamos una instancia de WiFiManager
    WiFi.mode(WIFI_STA);

    WiFiManager wm;

    bool res;
    res = wm.autoConnect("AutoConnectAP","password");
    
    if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
    } 
    else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
      // Una vez que se conecta exitosamente, imprimimos la dirección IP asignada
      Serial.println("Conexión establecida!");
      Serial.print("Dirección IP: ");
      Serial.println(WiFi.localIP());
      wifiPreviouslyConnected = true;
    }
  }
  else if (wifiConnect == 0 && wifiPreviouslyConnected) {
    // Si el pin StatusSW está en LOW, desconectamos el WiFi
    WiFi.disconnect();
    Serial.println("WiFi desconectado");

    wifiPreviouslyConnected = false;
  }
}

void firmwareUpdate(void)
{
  //WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  httpUpdate.setLedPin(Wifistatus, LOW);
  Serial.println("Updating firmware...");
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);
  Serial.println("Firmware updated!");

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  WiFiManager wm;

  wm.resetSettings();

  LoRa.setPins(RFM_CS, RFM_RST, RFM_DIO0); //Setea los pines de LoRa, sin contar miso, mosi y clk que lo realiza la libreria
  pinMode(Rele, OUTPUT); //declara el pin Rele como output
  pinMode(Wifistatus, OUTPUT); //declara el pin Estad Wifi como output
  pinMode(StatusSW, INPUT); //declara el pin SWITCH como input

  Serial.print("Conectando a LoRa..."); //busca inicializar LoRa, si no lo logra se queda suspendido
  while(!LoRa.begin(915E6))
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Listo LoRa");

  attachInterrupt(digitalPinToInterrupt(StatusSW), EstadoWIFI, CHANGE);
}

void loop() {

  ManejoWifi();
  if (wifiConnect == 1 && wifiPreviouslyConnected) {
    firmwareUpdate();
  }

  // Espera a recibir un mensaje LoRa
  ReceptorLoRa();

  digitalWrite(Rele, releState);
  /*
  int currentState = digitalRead(Rele);
  Serial.println("Estado actual:");
  Serial.println(currentState);

  // Cambia al estado opuesto
  int newState = !currentState;

  // Escribe el nuevo estado al relé
  digitalWrite(Rele, newState);
  Serial.println("Nuevo estado:");
  Serial.println(newState);
  */
  LedWifi();

  Serial.println("Versión 1");

  // Espera un tiempo antes de repetir el proceso
  delay(50);
}
