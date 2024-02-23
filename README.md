# Relay-LoRa

Este proyecto consiste en un Rele activado por lora para uso en redes de este estilo

El comunicador se encuetra trabajando a 915 Hz y se encuentra realizando mediciones constantes cada 10 milisegundos

Al recibir el mensaje por LoRA este se lee como un string que luego se deserializa en formato Json, de esta forma se identifica donde exista un en "Rele" y que valor lo sigue, si un 1 o un 0, esti es tomado como un int que por defecto se encuentra en estado 0, dependiendo de valor de el releState se realiza un digital Write en el pin de Rele

Al activar el switch de wifi realiza un autoconnect para que desde el celular se le de la red wifi, luego de conectarse a la red wifi verifica si hay una actualización de firmware en el repositorio

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

Wifi
* Output Led estado amarillo 33
* Input Switch de estado 34

# Funciones

## EstadoWIFI

Es una función manejada por interrupción, si cambia el estado del el switch de wifi activa un booleano que es wifiConnect, al desactivarse el switch se deseactiva la variable

## Led Wifi

Actualmente en desuso, servia para la etapa de debuggeo, cuando se encuentra el swtich de wifi activado y ya logro conectarse a wifi cambia su estado de prendido a apagado cada 0.5 segundos

## ReceptorLoRa

Utiliza el modulo lora y comienza a realizar una recepcción, el dato recibido de deserializa como un Json y se ubica en su interior el dato "R000" que representa al dispositivo, luego verifica si el valor que acompaña a este dato es 0 o 1 y dependiendo de esto prende o apaga el Rele, se tiene como indicador de estado además el LED verde en la tapa de el proyecto

## ManejoWifi

Esta función se encarga de gestionar el Autoconnect y desconect, dependiedno de si ya se encuentra conectado o no, y si la variable wifiConnect esta activada o no intenta conectarse con credenciales existentes, si no encuentra credenciales (caso base), genera una red Wifi "AutoConnectAP" con contraseña "password", al ingresar a esta red hay que luego ir a la IP del dispositivo, 192.168.4.1, al ir a esta dirección se vera una pagina en la cual se podran introducir los datos de la red wifi y su contraseña, si la conexión se realiza exitosamente se llamara a la función firmwareUpdate inmediatamente despues y el Led de estado de wifi, color amarillo comenzara a tintilar rapidamente, de no suceceder intente conectar nuevamente el wifi.

Esta funcion unicamente se llamara bajo la condición de que debido a la activación del switch se enceuntre habilitada la variable wifiConnect y que anteriormente no se encuentre habilitado con el find e no ejecutar mutiples veces este codigo.

## firmwareUpdate

Se intenta conectar al cliente, el cual sera github con el rootCA certificate, luego setea el led de update que sera Wifisattus, luego se intentara conectar el URL dado con el firmware nuevo, de funcionar correctamente se reiniciara la esp32 con su nuevo codigo.

Esta funcion sera llamada unicamente en el casod e que este activado el Wifi y ya se encuentre conectado.

# Consideraciones

El codigo actualmente se encuentra en un estado muy sencillo, no realiza una verificación de nodo por ejemplo para saber que se desea comunicar con el, ni tinee alguna especie de ahorro de bateria o semejante

El loop tiene un delay de 50 milisegundos, por lo que realiza constantemente todas las funciones declaradas bajo sus condiciones respectivas
