#include <Arduino.h>
#include <ArduinoJson.h>
#include <LoRa.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include "WiFiClientSecure.h"

#include <lmic.h>
#include <hal/hal.h>

 //Estos pines fueron utilizados en la prueba de concepto realizada en placa perforada con los componentes modulares, estan sujetos a cambios en la realización de una placa
#define RFM_CS       5
#define RFM_RST      17
#define RFM_DIO0     4
#define RFM_DIO1     12 //ESTE AUN DEBE ASIGNARSE
#define Rele     32
#define Wifistatus     33
#define StatusSW     34

#define ID       "R000"

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


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0xEE, 0xFA, 0x6F, 0xDC, 0xFC, 0x24, 0x61, 0x72 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x83, 0x59, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xF5, 0x92, 0xCD, 0xCC, 0x94, 0xFE, 0xD8, 0x4F, 0x94, 0x89, 0xD9, 0xD7, 0xD7, 0x94, 0x17, 0x36};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RFM_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM_RST,
    .dio = {RFM_DIO0, RFM_DIO1, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j)
{
  StaticJsonDocument<128> payload;
  String output;
  payload["id"]    = ID;
  payload["Estado RELE"]  = releState;

  serializeJson(payload, output);
	Serial.println(output);
	if (LMIC.opmode & OP_TXRXPEND)
	{
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
	else
	{
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, (unsigned char*) output.c_str(), sizeof(output)-1, 0);
      Serial.println(F("Packet queued"));
  }
    // Next TX is scheduled after TX_COMPLETE event.
}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


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

    //LoRa.setPins(RFM_CS, RFM_RST, RFM_DIO0); //Setea los pines de LoRa, sin contar miso, mosi y clk que lo realiza la libreria
    
    pinMode(Rele, OUTPUT); //declara el pin Rele como output
    pinMode(Wifistatus, OUTPUT); //declara el pin Estad Wifi como output
    pinMode(StatusSW, INPUT); //declara el pin SWITCH como input

    /*
    Serial.print("Conectando a LoRa..."); //busca inicializar LoRa, si no lo logra se queda suspendido
    while(!LoRa.begin(915E6))
    {
    Serial.print(".");
    delay(500);
    }
    Serial.println("Listo LoRa");
    */
    // LMIC init
    Serial.println("OS init");
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    Serial.println("lmic reset");
    LMIC_reset();
    Serial.println("lmic config");
    LMIC_setLinkCheckMode(0);
    //LMIC_setDrTxpow(DR_SF7,14);
    LMIC_selectSubBand(1);
    Serial.println("asing send job");
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
    Serial.println("ready");

    attachInterrupt(digitalPinToInterrupt(StatusSW), EstadoWIFI, CHANGE);
}

void loop() {

    ManejoWifi();
    if (wifiConnect == 1 && wifiPreviouslyConnected) {
    firmwareUpdate();
    }

    // Espera a recibir un mensaje LoRa
    //ReceptorLoRa();
    os_runloop_once();

    // Establecer la función de manejo de mensajes
    StaticJsonDocument<128> doc_rx;
    while (LMIC.dataLen > 0) {
        // Obtener el mensaje recibido
        String mensaje = String((char*)LMIC.frame, LMIC.dataLen);
        Serial.println(mensaje);

        DeserializationError error = deserializeJson(doc_rx, mensaje);  //Analiza lo recibido como un JSON para encontrar la posición de RELE
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
    };

    digitalWrite(Rele, releState);

    LedWifi();

    //Serial.println("Versión 1");

    // Espera un tiempo antes de repetir el proceso
    delay(50);
}
