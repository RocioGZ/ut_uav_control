# DEMO AROSA

Este repositorio contiene los pasos para ejecutar la demo de AROSA.

## Requisitos
Instalar en la Raspi:
- Ubuntu 20
- Ros noetic
- Mavros
- mocap-bridge (de grvc-utils)

## Instrucciones

### Terminal 1 (T1)

Lanza el puente de MoCap con OptiTrack:

```bash
roslaunch mocap_bridge optitack_bridge.launch
```

### Terminal 2 (T2)

Lanza el nodo para free-floating:

```bash
roslaunch ut_uav_control uavcontrol.launch
```
