#include <Arduino.h>
#include <ArduinoJson.h>
#include <LoRa.h>
 //Estos pines fueron utilizados en la prueba de concepto realizada en placa perforada con los componentes modulares, estan sujetos a cambios en la realización de una placa
#define RFM_CS       5
#define RFM_RST      17
#define RFM_DIO0     4
#define Rele     32

int releState = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(RFM_CS, RFM_RST, RFM_DIO0); //Setea los pines de LoRa, sin contar miso, mosi y clk que lo realiza la libreria
  pinMode(Rele, OUTPUT); //declara el pin Rele como output

  Serial.print("Conectando a LoRa..."); //busca inicializar LoRa, si no lo logra se queda suspendido
  while(!LoRa.begin(915E6))
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Listo LoRa");
}

void loop() {
  // Espera a recibir un mensaje LoRa
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
      if (doc_rx.containsKey("Rele")) {
        releState = doc_rx["Rele"];//Busca "Rele" para saber su estado
        Serial.print("Estado del rele: ");
        Serial.println(releState);
      } else {
        Serial.println("El JSON no contiene la clave 'Rele'");
      }
    }
  }
  digitalWrite(Rele, releState); //Asocia el ultimo valor registrado de releState a la salida, por defecto sera 0, cambia con el LoRa
  delay(10);
}
