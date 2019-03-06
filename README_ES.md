# ROS_Workshop_Py_CPP_ES
Taller en linea para aprender los aspectos basicos de ROS

## Instalacion

### Crear el espacio de trabajo y cambiar el directorio actual:

`mkdir -p catkin_ws/src && cd catkin_ws/src`

### Clonar el repositorio: 

`git clone https://github.com/TOTON95/ROS_Workshop_Py_CPP_ES.git`

### Cambiar el directorio actual a la raiz del directorio:

`cd ..`

### Seguir las instrucciones mencionadas en [Drawing Robot](https://github.com/TOTON95/Arduino_Drawing_Robot_OpenCV_OpenNI) (compilar y subir el archivo a la placa Arduino).

### En el archivo `ros_py_arm_sender.py` cambia la linea 37 al puerto serial de  Arduino (ej. /dev/ttyACM0, /dev/ttyUSB0).

### Compila el ejemplo:

`catkin_make`

### Ejecuta el experimento

`roslaunch ros_workshop_py_cpp ros_cpp_py.launch`


