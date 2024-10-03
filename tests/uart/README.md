Todos los comandos desde la Raspberry Pi hacia el Teensy son de 3 bytes.
Para iniciar se envía "000".
Para finalizar se envía "111".

Para asignar las acciones a los colores, el primer byte
corresponde a la acción ejecutada en verde, el segundo al azul, 
y el tercero al rojo.

Los códigos de acciones son los siguientes:
Frenar - 2
Retroceder - 3
Cambio de velocidad - 4

Por ejemplo, para retroceder en verde, frenar en azul y cambiar de velocidad
en rojo se envía "324", en ese orden.

Desde el Teensy a la Raspberry Pi se envían cadenas de largo variable, pues
en el microprocesador se tiene mucha más capacidad de manejar cadenas de texto.

El formato es:
"{segmentos verdes encontrados},{segmentos azules},{segmentos rojos}\n"