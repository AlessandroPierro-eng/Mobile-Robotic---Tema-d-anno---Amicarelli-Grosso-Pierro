# 🤖 Security Fleet - Mobile Robotics Project
Benvenuti nel repository del progetto di Robotica Mobile. 
Il progetto mira a sviluppare un sistema di sicurezza intercettivo basato su una flotta di 3 TurtleBot 4 all'interno di un magazzino simulato.

## 👥 Team
* Amicarelli
* Grosso
* Pierro

## 🎯 Obiettivi del Progetto
Il sistema è diviso in tre fasi principali:
1. **Fase 1 - Ambiente e Robotica:** Creazione del magazzino virtuale in Gazebo Harmonic e modifica dell'URDF del TurtleBot 4 per l'aggiunta di un giunto motorizzato e una telecamera.
2. **Fase 2 - Localizzazione e Visione:** Implementazione di un filtro EKF per fondere i dati odometrici con le letture degli AprilTag (tramite `apriltag-ros`) e integrazione di YOLOv8 per il riconoscimento visivo degli intrusi.
3. **Fase 3 - Navigazione (Nav2):** Gestione della navigazione multi-robot per l'esplorazione del magazzino e l'inseguimento del bersaglio.

## 🛠️ Tecnologie Utilizzate
* **OS:** Ubuntu 24.04 (tramite container Docker)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Simulatore:** Gazebo Harmonic
* **Computer Vision:** OpenCV & YOLOv8 (Ultralytics)

## 🚀 Come avviare l'ambiente di sviluppo
Abbiamo containerizzato l'intero workspace per garantire la massima riproducibilità. Assicurati di avere **Docker** installato e in esecuzione.