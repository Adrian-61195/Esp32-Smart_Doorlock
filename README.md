# ESP32 Smart Lock Hub

IoT smart lock with ESP32 — LCD & BLE control, RGB feedback, and temperature logging to InfluxDB Cloud.

## Function
The system acts as both a **smart door lock** and an **IoT telemetry node**:

- Lock/unlock control via physical buttons, potentiometer PIN entry, or Bluetooth commands  
- Visual feedback through a 16×2 LCD and RGB LEDs  
- Servo motor for the locking mechanism  
- Temperature sensing with a thermistor, logged to InfluxDB Cloud  
- Time synchronization over NTP for accurate timestamps  
- Data pipeline ready for analysis and visualization with **Grafana dashboards**, enabling real-time monitoring of temperature trends, lock activity, and IoT device health  
