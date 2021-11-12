# Investigación

Los sensores que componen al robot **Robotis Turtlebot3 Waffle* son los siguientes:

+ Sensor de distancia láser 360 LDS-01

Es un láser 2D capaz de sensar a 360° que obtiene una serie de datos del entorno en donde se encuentra el robot para hacer localización y mapeo simultáneos y navegación. Su distancia de detección va de 120 a 3500 [mm] y la resolución de barrido angular es de 1° [[1]](../readme.md#referencias#1).

La transmisión de datos es realizada mediante el tópico `/scan`, que muestra información como los ángulos de barrido del sensor, las distancias mínima y máxima medibles así como el rango de distancias a objetos del entorno por grado [[2]](../readme.md#referencias#2).

![sensorlaser](images/lds.png "Sensor de distancia láser 360 LDS-01")  

+ Cámara Raspberry Pi v2.1

Es un módulo que cuenta con una cámara de 8 megapixeles que puede ser utilizado para tomar fotografías o video. Cuenta con distintos efectos para modificar la imagen resultante en tiempo real [[3]](../readme.md#referencias#3).

La transmisión de datos es realizada mediante los tópicos `/camera`, que muestran información como la imagen actual o la información de la cámara. Asimismo, cuenta con servicios `/camera` que pueden modificar la información de la cámara o su calibración [[4]](../readme.md#referencias#4).

![camara](images/Pi-Camera-front.jpg "Cámara Raspberry Pi v2.1")

[Volver](../)