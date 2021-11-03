# Tarea 01

## Contenido

+ [Objetivo](#objetivo)
+ [Introducción](#introducción)
+ [Desarrollo](#desarrollo)
+ [Conclusiones](#conclusiones)
+ [Autor](#autor)
+ [Referencias](#referencias)

## Objetivo

Hacer que un robot modifique su comportamiento a partir de los siguientes comandos de texto:

+ Avanza [velocidad lineal]

+ Gira [velocidad angular]

+ Detente

Restricciones:
+ El nombre del paquete de ROS generado debe ser **robot_comm**.
+ El robot a operar debe ser **TurtleBot3** en cualquiera de sus versiones (waffle, waffle_pi o burger).
+ Sólo se pueden usar Publicadores y Subscriptores.

## Introducción

Dada la forma en la que está estructurado ROS al implementar el modelo de patrones de diseño, es posible crear una amplia variedad de programas de robótica implementando el patrón de publicador y suscriptor. Gracias a que se cuenta con la libertad de elegir a qué tópico puede publicar o suscribirse uno o varios nodos, se puede realizar todo tipo de simulaciones con distintos propósitos. Cabe mencionar que el modelo de patrones de diseño no solo es dirigido para robótica, sino que puede encontrar aplicaciones muy distintas y hasta de una mayor complejidad, llegando a involucrarse con entornos industriales o de IoT.

## Desarrollo

Se describirá por partes el programa realizado.

Lo primero es inicializar el nodo que se utilizará para el programa y establecer tanto el publicador como el suscriptor que se utilizarán.

```python
rospy.init_node('robot_comm')
sub=rospy.Subscriber('odom',Odometry,command_callback)
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
rate=rospy.Rate(2)
```

El tópico `cmd_vel` es del tipo de dato `Twist`, por lo cual se declarará una variable de este tipo para interpretar los comandos del usuario y poder aplicar estos cambios a la simulación.

```python
velocidad=Twist()
```

El suscriptor declarado requiere de una función de callback, la cual será definida arriba del código mencionado anteriormente.

```python
def command_callback(msg):
    comando=raw_input("Ingrese comando: ")
    if comando[:7]=="Avanza ":
        velocidad.linear.x=float(comando[7:])
    elif comando[:5]=="Gira ":
        velocidad.angular.z=float(comando[5:])
    elif comando=="Detente":
        velocidad.linear.x=0
        velocidad.angular.z=0
    else:
        rospy.loginfo("Error")
    pub.publish(velocidad)
```

Se pide al usuario el comando que desee mandar al robot, empleando `raw_input` para recibir la información como una cadena. Posteriorente se evalúa el comando ingresado de acuerdo a las especificaciones establecidas en el objetivo. Se extrae el número de la cadena dependiendo de si se pide una velocidad lineal o angular contando la cantidad de caracteres de la cadena y se asocia a su respectivo parámetro de la variable declarada de tipo `Twist`, empleando un cast para convertir el dato a un número de tipo flotante [[1]](#1). Si se escribe el comando con algún error o si no se escribe un espacio entre la palabra y el número, se mandará un mensaje de error al usuario. Al final, el publicador publicará la variable al tópico `cmd_vel` para implementarse en la simulación.

## Conclusiones

Con la realización de esta actividad se pudo comprender el concepto de publicador y suscriptor para un programa de ROS. La realización del programa solicitado fue algo sencilla considerando que se realizó un ejercicio previamente en clase donde se hizo uso de publicadores y suscriptores. Es importante ubicar los tipos de datos asociados a los tópicos para saber qué variables serán necesarias y cuáles de sus parámetros serán modificados para implementar correctamente la publicación de un nodo a un tópico.

## Autor

**Autor** Talavera Montenegro Antonio [GitHub profile](https://github.com/atalaveram)

## Referencias

<a id="1">[1]</a> ROS Documentation. (2021, Jan. 15). *geometry_msgs/Twist Documentation* [Online]. Available: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
