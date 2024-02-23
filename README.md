# Relay-LoRa

Este proyecto consiste en un Rele activado por lora para uso en redes de este estilo

El comunicador se encuetra trabajando a 915 Hz y se encuentra realizando mediciones constantes cada 10 milisegundos

Al recibir el mensaje por LoRA este se lee como un string que luego se deserializa en formato Json, de esta forma se identifica donde exista un en "Rele" y que valor lo sigue, si un 1 o un 0, esti es tomado como un int que por defecto se encuentra en estado 0, dependiendo de valor de el releState se realiza un digital Write en el pin de Rele

# Pines asociados

Para el manejo del modulo LoRa y el Rele se tienen considerados los pines de SPI en primera instancia de forma que se consideran los pines
* MISO 19
* MOSI 23
* SCK 18
* CS 5
* RST 17
* DIO0 4

Rele
* Output Rele 32

# Consideraciones

El codigo actualmente se encuentra en un estado muy sencillo, no realiza una verificación de nodo por ejemplo para saber que se desea comunicar con el, ni tinee alguna especie de ahorro de bateria o semejante
