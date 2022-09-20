# Lab2: Robótica de Desarrollo, Intro a ROS 
## Integrantes: Allan Giordy Serrato Gutierrez y Juan Fernando Ramirez Montes

## Objetivo
Escribir un código que permita operar una tortuga del paquete turtlesim de ROS con el teclado, siguiendo las siguientes especificaciones:
- Se debe mover hacia adelante y hacia atrás con las teclas W y S
- Debe girar en sentido horario y antihorario con las teclas D y A.
- Debe retornar a su posición y orientación centrales con la tecla R
- Debe dar un giro de 180° con la tecla ESPACIO

## Resultados 
En el siguiente video se muestra el resultado obtenido por el equipo de trabajo para este laboratorio. La implementación se realizo por medio de un script de python integrado a la arquitectura ROS del paquete turtlesim.

https://user-images.githubusercontent.com/62154397/191142231-cfd46464-0bbc-44ff-9221-807c0d7c70dd.mp4

## Procedimiento

### Creación del workspace
Primero se creó la carpeta *catkin_ws* para crear el workspace catkin del proyecto, que es donde todos los paquetes ROS fueron guardados y modificados. Luego se compila toda la carpeta para activar el workspace. Las siguientes lineas de codigo realizan este procedimiento.

```console
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
```
### Descargar turtlesim
Se clonó el repo [hello_turtle](https://github.com/felipeg17/hello_turtle.git) dentro de la carpeta src del workspace creado para descargar el paquete turtlesim en el proyecto.

```console
cd catkin_ws/src
git clone https://github.com/felipeg17/hello_turtle.git
```
### Script myTeleopKey.py
Una vez descargado el paquete, se procede a escribir el script en python3 donde se crearan los nodos publisher y los key listeners de estos para mover la tortuga, teniendo en cuenta que el nodo del proyecto descargado se llama */turtle1*. A continuación se describe el código escrito en este script.

#### Importación de Librerias.
```console
from pynput.keyboard import Key, Listener
import rospy
from getkey import getkey, keys
from geometry_msgs.msg import Twist 
import numpy as np
import math as mt
from turtlesim.srv import TeleportAbsolute,TeleportRelative
from std_srvs import srv
```

#### Publisher para el tópico */turtle1/cmd_vel*
```console
def pubVel(vel_x, ang_z, t):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('velPub', anonymous=False)
    vel = Twist()
    vel.linear.x = vel_x
    vel.angular.z = ang_z
    #rospy.loginfo(vel)
    endTime = rospy.Time.now() + rospy.Duration(t)
    while rospy.Time.now() < endTime:
        pub.publish(vel)
```
El topico */cmd_vel* es de tipo *geometry_msgs/Twist* que esta compuesto por dos vectores 3x1 con componentes en *x,y* y *z*. Un vector de velocidad lineal y un vector de velocidad angular. 

Para ir hacia adelante solo se necesita tener velocidad lineal en el componente x mayor a cero, y para ir hacia atras la misma velocidad pero con signo opuesto.

Por otro lado, para girar en sentido antihorario solo se necesita una velocidad angular en el componente z positiva, y para girar en el sentido opuesto solo se necesita invertir el signo de esta.

Por eso vemos que esta función solo cambia los parametros *vel.linear.x* y *vel.angular.z* del tópico.

#### Servicio para movimiento absoluto

```console
def teleport(x, y, ang):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleportA = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp1 = teleportA(x, y, ang)
        #print('Teleported to x: {}, y: {}, ang: {}'.format(str(x),str(y),str(ang)))
    except rospy.ServiceException as e:
        print(str(e))
```
El servicio del nodo *turtle1* llamado *turtle1/TeleporAbsolute(x,y,theta)* mueve a la tortuga a las coordenadas absolutas en pantalla *(x,y)* y le da una orientación *theta* respecto al eje x. Por lo que para mover a la centro a la tortuga se le dan las coordenas (5.5,5.5) y uan orientación de 0° respecto a la horizontal.

#### Servicio para movimiento relativo
``` console
def move(x, ang):
    rospy.wait_for_service('/turtle1/teleport_relative')
    try:
        moveA = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
        resp1 = moveA(x, ang)
        #print('Relatively teleported to x: {}, ang: {}'.format(str(x),str(ang)))
    except rospy.ServiceException as e:
        print(str(e))
```
El servicio del nodo *turtle1* llamado *turtle1/TeleporRelative(linear,angular)* mueve a la tortuga una distancia *lineal* y una distancia *angular* desde su posición actual. En este caso para rotar la tortuga 180° solo necesitaremos darle un desplazamiento angular de 180° respecto a su posición actual.

#### Listeners de teclado
``` console
def on_press(key):
    key =getkey()
    if key == 'a':
        #rotacion antihoraria,angz+
        pubVel(0,1,0.5)
    elif key == 'd':
        #rotacion horaria ,angz-
        pubVel(0,-1,0.5)
    elif key == 'w':
        #hacia adelante ,velx+
        pubVel(1,0,0.5)
    elif key == 's':
         #hacia atras, velx-
        pubVel(-1,0,0.5)    
    elif key == 'r':
        #R para volver a posición y rotacion inicial
        teleport(5.5,5.5,0)
    elif key == ' ':
        #espacio para rotar 180°
        move(0,mt.pi)

def on_release(key):
    if key == Key.esc:
        return False
```
Ya con las funciones establecidas se crea un listener on_press(key) para que llame a las funciones cuando cierta tecla sea presionada, cada tecla se especifica por medio de una estrutura *else if*. Entonces se programaron las siguientes acciones:

- Cuando la tecla *A* sea presionada se publicará un tópico *Twist()* con una velocidad angular en z de +1 para rotación antihoraria.
- Cuando la teccla *D* sea presionada se publicará un tópico *Twist()* con una velocidad angular en z de -1 para rotación horaria.
- Cuando la telca *W* sea presionada se publicará un tópico *Twist()* con una velocidad lineal en x de +1 para ir hacia adelante.
- Cuando la telca *S* sea presionada se publicará un tópico *Twist()* con una velocidad lineal en x de -1 para ir hacia atras.
- Cuando la tecla *R* sea presionada se llamará al servicio *TeleportAbsolute()* para que lleve a la tortuga a las coordenadas *(5.5,5.5)* y le de una orientación de 0° respecto a la horizontal de la pantalla.
- Cuando la tecla *Espacio* sea presionada se llamará al servicio *TeleportRelative()* con un desplazamiento lineal nulo y un desplazamiento angular de 180°.

Todos los tópicos *Twist()* se llaman con una duración de 0.5s.
Se añade el listener para la tecla *esc* para poder salir del evento de movimiento por teclado en la consola.


#### Llamado periodico del listener en main
``` console
if __name__ == '__main__':
    pubVel(0,0,0.1)
    try:
        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except rospy.ROSInterruptException:
        pass
```
Por último, se crea un if que siempre se llamara si el main() de la simulación esta corriendo (se llama siempre que el nodo turtle1 este corriendo) con una llamada al listener de teclado creado anteriormente y se le agrega un catcher para la excepción *ROSInterruptException* que permitirá dejar de correr el código cuando se oprima *crt+C* en la consola o cuando se deje de correr la simulación del a tortuga.

### Añadir el script al workspace
Una vez escrito el script de python se deben seguir los siguientes pasos para que este sea reconocido por el paquete *turtlesim* instalado y pueda comunicarse con el nodo que crea sin problemas.

El **primer paso** es guardar el archivo * myTeleopKey.py* en la carpeta *catkin_ws/src/hello_turtle/scrpits* para que quede guardada dentro del directorio del workspace creado para este laboratorio.

El **segundo paso** es incluir el script en el apartado *catkin_install_python* del archivo *CMakeLists.txt* que se encuentra en la capeta *catkin_ws/src/hello_turtle/*. Dicho apartado lucirá así cuando el script ya sea añadido: 

``` console
...
catkin_install_python(PROGRAMS
  scripts/turtlePos.py
  scripts/turtleSpawn.py
  scripts/turtleSub.py
  scripts/turtleVel.py
  scripts/myTeleopKey.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
...
```
El **tercer paso** es volver a generar el workspace para que asimile los cambios realizados al archivo *CMakeLists.txt*. Luego, toca volver a añadir al workspace al entorno ROS llamando al archivo setup. Las tres lineas de código siguientes hacen este proceso.
``` console
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
```
Una vez se hallan realizado estos tres pasos el paquete *turtlesim* ya quedará listo para ser corrido en consola junto con el script de python.

### Correr turtleSim
Para poder correr el paquete *turtlesim* en la arquitectura ROS ahi que abrir tres terminales en la consola de linux.

#### Primera terminal
En la primera terminal se escribe la siguiente linea de código para empezar a correr ROS.
```console
roscore
```
#### Segunda terminal
En la segunda terminal se escribe la siguiente linea de codigo para inciar el nodo *turtlesim_node* que corre la simulación de la tortuga.
```console
rosrun turtlesim turtlesim_node
```
#### Tercera terminal
En la tercela terminal se debe correr el script de python creado anteriormente *myTeleopKey.py* para poder mover la tortuga como se describe en el objetivo del proyecto. Entonces se escribe el comando:
```console
rosrun hello_turtle myTeleopKey.py
```
Una vez se hallan ejecutado estas lineas en cada una de las terminales de forma secuencial se obtendra una interfaz como la de la imagen, donde la tortuga será movida por las teclas mencionadas al inicio del archivo.

![TerminalTurtlesim](https://user-images.githubusercontent.com/62154397/191153524-76863a47-f67f-4558-8cfc-6897d8b97522.png)

Cabe recordar que este último paso se ve también ilustrado en el video presentado como resultado de este laboratorio al inicio del documento.

## Referencias
- https://github.com/fegonzalez7/rob_unal_clase2.git
- https://drive.google.com/file/d/19UOE_eI-ob2ZymNHWFrYgrxLQfgOon43/view
- https://github.com/felipeg17/hello_turtle
- http://wiki.ros.org/ROS/Tutorials/CreatingPackage
- http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel
- http://wiki.ros.org/turtlesim
- https://pynput.readthedocs.io/en/latest/keyboard.html#pynput.keyboard.KeyCode 
- https://pypi.org/project/getkey/
