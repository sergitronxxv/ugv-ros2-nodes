# UGV ROS2 Nodes

Nodos de **ROS2 Humble** desarrollados para el control y la monitorización de un vehículo terrestre no tripulado (UGV), comunicados con una controladora de vuelo Ardupilot/Ardurover a través del paquete **MavRos**.

Este código forma parte del Trabajo de Fin de Grado *"Desarrollo de un UGV autónomo con control remoto y navegación basada en ROS2"*.

## Nodos incluidos

### `vel_calc.py`
Nodo de control de velocidad para el modo guiado. A partir de la posición GPS del vehículo y de la lista de waypoints de la misión activa, calcula la velocidad lineal y angular necesarias para alcanzar el siguiente waypoint, y las publica en el tópico `/mavros/setpoint_velocity/cmd_vel`.

Características principales:
- Estimación de rumbo mediante fusión entre el curso GPS (course over ground) y la velocidad angular del giroscopio, sin depender de la brújula.
- Control PI de velocidad lineal con rampa de aceleración/frenado y saturación anti-windup.
- Rutina de arranque en frío mientras no existe una referencia de rumbo fiable.
- Detección de misiones nuevas frente a misiones reenviadas por MavRos, evitando reinicios de progreso innecesarios.
- Gestión del cambio de modo (GUIDED/MANUAL) y del ciclo de vida de la misión mediante los servicios `SetMode`, `WaypointPull` y `WaypointClear`.

### `qgc_mission_status_manual.py`
Nodo de monitorización y control manual, empleado durante las pruebas de campo con QGroundControl.

Características principales:
- Registro periódico del estado del vehículo: posición GPS, orientación (IMU), canales de radiocontrol y velocidad respecto al suelo.
- Seguimiento del estado de la misión cargada (waypoints, último waypoint alcanzado).
- Control manual por teclado (W/A/S/D + barra espaciadora) mediante lectura de la terminal en modo "raw" (módulo `termios`), publicando órdenes `ManualControl` a 10 Hz.

## Dependencias

- ROS2 Humble Hawksbill
- [mavros](https://github.com/mavlink/mavros)
- Python 3 (`rclpy`, `termios` en el caso de `qgc_mission_status_manual.py`)

## Documentación relacionada

El funcionamiento detallado de estos nodos se describe en el apartado 5.2.3 y en el Anexo B de la memoria del TFG.
