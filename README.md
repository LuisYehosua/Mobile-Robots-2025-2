# Mobile-Robots-2024-2
Software para el curso "Robots Móviles" semestre 2024-2, FI, UNAM

## Requerimientos

* Ubuntu 20.04 https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2024-2
* $ cd Mobile-Robots-2024-2
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2
* $ echo "source ~/Mobile-Robots-2024-2/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2024-2/catkin_ws/devel/setup.bash
* $ roslaunch surge_et_ambula movement_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un visualizador como el siguiente:
![rviz](https://github.com/mnegretev/Mobile-Robots-2024-2/blob/main/Media/rviz.png)

Un ambiente simulado como el siguiente:
![gazebo](https://github.com/mnegretev/Mobile-Robots-2024-2/blob/main/Media/gazebo.png)

Y una GUI como la siguiente:
![GUIExample](https://github.com/mnegretev/Mobile-Robots-2024-2/blob/main/Media/gui.png)


## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
