# Tarea 02

## Contenido

+ [Objetivo](#objetivo)
+ [Introducción](#introducción)
+ [Desarrollo](#desarrollo)
+ [Conclusiones](#conclusiones)
+ [Autor](#autor)
+ [Referencias](#referencias)

## Objetivo

+ ***Investigación.*** Investigar los diferentes sensores que componen al robot **Robotis Turtlebot3 Waffle** y su transmisión de datos en ROS (nodos, tópicos, servicios, simulaciones).

+ ***Simulación.*** Mover al robot de su posición inicial a un punto dado de coordenadas (X, Y).

Restricciones:

+ El reporte de investigación debe ser redactado en uno o varios documentos de **markdown** en la carpeta **docs** y referenciados a través del archivo `README.md`

+ El nombre del paquete de ROS generado debe ser **robot_comm**, usado en la tarea anterior.

+ El robot a operar debe ser **TurtleBot3** *Waffle*.

+ Para los obstáculos se puede usar la técnica vista en clase o algún mundo de su preferencia.

+ El sentido de giro para evitar el obstáculo puede ser arbitrario: siempre horario, siempre anti-horario o algún otro parámetro que deseen emplear para la decisión del robot.

+ Se pueden usar Publicadores, Suscriptores y Servicios.

## Introducción



## Desarrollo

La investigación realizada está en el siguiente [documento](docs/investigacion.md).

Se describirá por partes el programa realizado.

Se creará una clase llamada `RobotMover`, que se encargará del movimiento del robot. Dentro del constructor se declararán las siguientes variables:

+ `_distance_to_point`: un número de punto flotante que almacenará la distancia que hay entre la posición actual del robot y el punto de destino del robot.
+ `_current_position`: un dato de tipo `Point` que almacenará las coordenadas de la posición actual del robot.
+ `_current_orientation`: un dato de tipo `Quaternion` que almacenará los datos correspondientes a la orientación actual del robot.
+ `_destination`: un dato de tipo `Point` que almacenará las coordenadas del punto de destino del robot. Dichas coordenadas serán pedidas al usuario.
+ `_arrived_to_destination`: un booleano que funcionará como condición para que el robot continúe o detenga su movimiento dependiendo de la distancia que haya entre la posición actual del robot y el punto de destino.
+ `_velocity`: un dato de tipo `Twist` que almacenará los datos correspondientes a los cambios de velocidad que debe realizar el robot durante la simulación, que será publicado al tópico `/cmd_vel`
+ `_angle_sections`: una lista de datos que almacenará las distancias del robot a los objetos del entorno por cada grado empleando el LIDAR integrado.

```python
class RobotMover(object):
    def __init__(self):
        self._distance_to_point=Float64()
        self._distance_to_point.data=0.0
        self._current_position=Point()
        self._current_orientation=Quaternion()
        self._destination=Point()
        self._destination.x=float(input('Ingrese coordenada en x: '))
        self._destination.y=float(input('Ingrese coordenada en y: '))
        self._arrived_to_destination=False
        self._velocity=Twist()
        self._angle_sections={}
        self.get_init_position_and_orientation()
        self.robot_moved_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.obstacle_avoid_sub=rospy.Subscriber('/scan',LaserScan,self.obstacle_avoid_callback)
        self.robot_moved_sub=rospy.Subscriber('/odom',Odometry,self.odom_callback)
```
Se manda llamar una función propia de la clase que obtendrá la posición y la orientación iniciales del robot. En dicha función se utilizará la función `wait_for_message` para crear un suscriptor temporal que recibirá el primer mensaje de posición y orientación del tópico `/odom`. Nuevamente se utilizará `wait_for_message` para obtener un primer mensaje, pero ahora de las distancias a los objetos del entorno. Este mensaje se recibirá del tópico `/scan`.

Una vez recibidos los mensajes, se asignarán los valores obtenidos de posición, orientación y distancias a las variables declaradas en el constructor. Para `_angle_sections`, se dividió la zona de detección del robot en 6 secciones, 5 delanteras y una trasera. El objetivo de las zonas delanteras es ubicar qué extremo del robot está más cercano a un objeto para determinar el giro adecuado para esquivarlo y el objetivo de la zona trasera es establecer una distancia a la cual el robot debe alejarse del objeto para después continuar con su trayectoria original.

```python
    def get_init_position_and_orientation(self):
        data_odom=None
        while data_odom is None:
            try:
                data_odom=rospy.wait_for_message('/odom',Odometry,timeout=1)
            except Exception as e:
                rospy.logerr(e)
        data_scan=None
        while data_scan is None:
            try:
                data_scan=rospy.wait_for_message('/scan',LaserScan,timeout=1)
            except Exception as e:
                rospy.logerr(e)
        self._current_position.x=data_odom.pose.pose.position.x
        self._current_position.y=data_odom.pose.pose.position.y
        self._current_position.z=data_odom.pose.pose.position.z
        self._current_orientation.x=data_odom.pose.pose.orientation.x
        self._current_orientation.y=data_odom.pose.pose.orientation.y
        self._current_orientation.z=data_odom.pose.pose.orientation.z
        self._current_orientation.w=data_odom.pose.pose.orientation.w
        self._angle_sections={
        'right':  min(min(data_scan.ranges[270:306]),10),
        'fright': min(min(data_scan.ranges[306:342]),10),
        'front':  min(min(min(data_scan.ranges[0:18]),min(data_scan.ranges[342:359])),10),
        'fleft':  min(min(data_scan.ranges[19:54]),10),
        'left':   min(min(data_scan.ranges[55:90]),10),
        'back':   min(min(data_scan.ranges[91:269]),10)}
```

Posterior a la función `get_init_position_and_orientation` se tiene declarado el publicador `robot_moved_pub` que se encargará de publicar la información de la variable `_velocity` al tópico `/cmd_vel`.

Se tienen dos suscriptores, uno que recibirá información de tipo de dato `LaserScan` del tópico `/scan` y otro que recibirá información de tipo de dato `Odometry` del tópico `/odom`. Cada uno de ellos contará con su correspondiente función de callback.

El único objetivo de la función de callback del suscriptor del tópico `/scan` es actualizar las distancias, manteniendo las mismas secciones establecidas.

```python
    def obstacle_avoid_callback(self,msg):
        self._angle_sections={
        'right':  min(min(msg.ranges[270:306]),10),
        'fright': min(min(msg.ranges[306:342]),10),
        'front':  min(min(min(msg.ranges[0:18]),min(msg.ranges[342:359])),10),
        'fleft':  min(min(msg.ranges[19:54]),10),
        'left':   min(min(msg.ranges[55:90]),10),
        'back':   min(min(msg.ranges[91:269]),10)}
```

La función de callback asociada al suscriptor del tópico `/odom` se encargará de actualizar la posición y la orientación del robot, recalcular la distancia entre la posición actual del robot y el punto de destino, evaluar la distancia entre el robot y los objetos para modificar el comportamiento del robot y por último, publicar la variable `_velocity` con la nueva información al tópico `/cmd_vel`. Además, imprimirá en pantalla las coordenadas en x y en y de la posición actual del robot, así como la distancia que hay entre la posición del robot y el punto de destino.

```python
    def odom_callback(self,msg):
        new_position=msg.pose.pose.position
        new_orientation=msg.pose.pose.orientation
        self._distance_to_point.data=self.calculate_distance(new_position,self._destination)
        self.update_current_position(new_position)
        self.update_current_orientation(new_orientation)
        rospy.loginfo('x={:.3f} m, y={:.3f} m, distancia={:.3f} m'.format(new_position.x,new_position.y,self._distance_to_point.data))
        self.avoid_obstacles()
        self.robot_moved_pub.publish(self._velocity)
```

Las funciones `update_current_position` y `update_current_orientation` asignarán los nuevos componentes de la posición y de la orientación del robot a las variables `_current_position` y `_current_orientation`, respectivamente. La función `calculate_distance` calculará la distancia que hay entre dos puntos utilizando la resta de las componentes en x y en y y la función `hypot` para calcular la raíz cuadrada.

```python
    def update_current_position(self,new_position):
        self._current_position.x=new_position.x
        self._current_position.y=new_position.y
        self._current_position.z=new_position.z
    def update_current_orientation(self,new_orientation):
        self._current_orientation.x=new_orientation.x
        self._current_orientation.y=new_orientation.y
        self._current_orientation.z=new_orientation.z
        self._current_orientation.w=new_orientation.w
    def calculate_distance(self,new_position,old_position):
        x2=new_position.x
        x1=old_position.x
        y2=new_position.y
        y1=old_position.y
        dist=math.hypot(x2-x1,y2-y1)
        return dist
```
La función `avoid_obstacles` se encargará de establecer las velocidades del robot dependiendo de las distancias entre el robot y los objetos y de la distancia entre la posición del robot y el punto de destino. La condición inicial para iniciar la evaluación es que el robot no haya llegado al punto de destino, representado con la variable booleana `_arrived_to_destination` declarada en el constructor. La segunda condición es que el robot esté a una distancia mayor a 1 [cm] del punto de destino, para que de esta forma pueda frenar y establecerse en una zona muy cercana al punto de destino.

```python
    def avoid_obstacles(self):
        if not self._arrived_to_destination:
            if self._distance_to_point.data>0.01:
```

Si se cumplen las dos condiciones, se procederá a evaluar la velocidad del robot dependiendo de las distancias del robot a los objetos del entorno. La distancia mínima que se evaluará para todos los casos será de 40 [cm]. El segundo caso se cumplirá si se detecta una distancia menor a la mínima en la sección delantera del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                if self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 2 - front'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El tercer caso se cumplirá si se detecta una distancia menor a la mínima en la sección delantera derecha del robot, deteniendo el robot y realizando un giro en sentido antihorario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 3 - fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
```

El cuarto caso se cumplirá si se detecta una distancia menor a la mínima en la sección delantera izquierda del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 4 - fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El quinto caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera y delantera derecha del robot, deteniendo el robot y realizando un giro en sentido antihorario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 5 - front and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
```

El sexto caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera y delantera izquierda del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 6 - front and fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El séptimo caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera izquierda, delantera y delantera derecha del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 7 - front and fleft and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El octavo caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera izquierda y delantera derecha del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 8 - fleft and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```
El noveno caso se cumplirá si se detecta una distancia menor a la mínima en la sección derecha del robot, deteniendo el robot y realizando un giro en sentido antihorario para evitar el obstáculo.

```python
                elif self._angle_sections['right'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 9 - right'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
```

El décimo caso se cumplirá si se detecta una distancia menor a la mínima en la sección izquierda del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['left'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] > 0.4:
                    state_description = 'case 10 - left'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El décimo primer caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera derecha y derecha del robot, deteniendo el robot y realizando un giro en sentido antihorario para evitar el obstáculo.

```python
                elif self._angle_sections['right'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 11 - right and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
```

El décimo segundo caso se cumplirá si se detecta una distancia menor a la mínima en las secciones delantera izquierda e izquierda del robot, deteniendo el robot y realizando un giro en sentido horario para evitar el obstáculo.

```python
                elif self._angle_sections['left'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4:
                    state_description = 'case 12 - left and fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
```

El primer caso se cumplirá si para las 5 secciones frontales se detecta una distancia mayor a la mínima. En este caso, se obtendrá primero el ángulo en el que está orientado el robot en el eje z utilizando la tercera componente del vector resultante de utilizar la función `euler_from_quaternion` y luego el ángulo al cual debería apuntar el robot, determinado con la función `atan2` y la diferencia de las componentes en x y en y de la posición actual del robot y del punto de destino.

```python
                else:
                    state_description = 'case 1 - normal'
                    theta_z=euler_from_quaternion([self._current_orientation.x,self._current_orientation.y,self._current_orientation.z,self._current_orientation.w])[2]
                    angle=atan2(self._destination.y-self._current_position.y,self._destination.x-self._current_position.x)
```

Se evaluará si se detecta una distancia menor a la mínima en la sección trasera del robot. Si se cumple este caso, se hará avanzar al robot hasta que se haya alejado lo suficiente del obstáculo. Al no cumplirse el caso, se evaluará si la diferencia entre los dos ángulos obtenidos anteriormente es mayor a 0.1 [rad], para que de esta forma el robot pueda frenar y establecerse en el ángulo que apunta al punto de destino. Si el ángulo del punto de destino es menor al ángulo al cual apunta actualmente el robot, realizará un giro en sentido horario. Si es mayor, realizará un giro en sentido antihorario. Una vez que la diferencia de ángulos sea menor a la establecida, se hará avanzar al robot en la dirección actual hacia el punto de destino. Además, se imprimen en pantalla las distancias de cada uno de las 5 secciones delanteras y el caso actual de la evaluación de evasión de obstáculos del robot.

```python
                    if self._angle_sections['back'] < 0.4:
                            self._velocity.linear.x=0.15
                            self._velocity.angular.z=0.0
                    else:
                        if abs(angle-theta_z) > 0.1:
                            if angle<theta_z:
                                self._velocity.linear.x=0.0
                                self._velocity.angular.z=-0.3
                            else:
                                self._velocity.linear.x=0.0
                                self._velocity.angular.z=0.3
                        else:
                            self._velocity.linear.x=0.15
                            self._velocity.angular.z=0.0
                    rospy.loginfo('front={:.3f} m, fleft={:.3f} m, right={:.3f} m, fright={:.3f} m, left={:.3f} m'.format(self._angle_sections['front'],self._angle_sections['fleft'],self._angle_sections['right'],self._angle_sections['fright'],self._angle_sections['left']))
                rospy.loginfo(state_description)
```

Una vez que la distancia entre la posición actual del robot y el punto de destino es menor a la establecida, se detendrá el robot y se pondrá en verdadero la variable `_arrived_to_destination` para que se detenga la evaluación de evasión de obstáculos. La razón por la que existe esta condición es que el robot al frenar por completo puede detenerse en una posición a una distancia mayor a la establecida de 1 [cm], lo cual iniciaría nuevamente la evaluación que movería al robot en la zona cercana al punto de destino.

Estando en verdadero la variable `_arrived_to_destination`, la primera condición de toda la función no se cumplirá, por tanto, ya no se realizará la evaluación de evasión de obstáculos y el robot permanecerá en su lugar. Se imprimirá un mensaje en pantalla indicando que el robot ha llegado a su destino.

```python                
            else:
                self._velocity.linear.x=0.0
                self._velocity.angular.z=0.0
                self._arrived_to_destination=True
                rospy.loginfo('El robot ha llegado a su destino')
        else:
            self._velocity.linear.x=0.0
            self._velocity.angular.z=0.0
            rospy.loginfo('El robot ha llegado a su destino')
```
Por último, se definirá una función que mantenga ejecutando el programa hasta que se cierre manualmente y se procederá a inicializar el nodo a utilizar, a instanciar la clase RobotMover creada y a mantener el programa en ejecución.

```python
    def publish_moved_robot(self):
        rospy.spin()
if __name__=='__main__':
    rospy.init_node('robot_movement_node')
    rob_mov=RobotMover()
    rob_mov.publish_moved_robot()
```

## Conclusiones



## Autor

| Iniciales  | Descripción |
| -:| - |
| **AOJ** | Amilpa Olivera Joseph [GitHub profile](https://github.com/Josephamilpaolivera) |
| **GSJF**  | Guzmán Silva Jesús Fernando [GitHub profile]() |
| **TMA** | Talavera Montenegro Antonio [GitHub profile](https://github.com/atalaveram) |

## Referencias

<a id="1">[1]</a> ROS Documentation. (2021, Jan. 15). *geometry_msgs/Twist Documentation* [Online]. Available: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html