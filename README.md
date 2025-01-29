# 🏠 README.md

## 🔹 Project Overview  
A 4WD rover designed for radiation mapping, using a **UV sensor** as a proof of concept. The system collects and logs environmental data while navigating by using X-Bee.
![IMG_20241204_121347824_HDR](https://github.com/user-attachments/assets/8070fac8-6dca-4bc6-946c-d960c50a18de)

## 🔹 Features  
✔️ Radiation-level simulation using a UV sensor  
✔️ Ultrasonic sensors for obstacle detection  
✔️ GPS with RTC for real-time location tracking  
✔️ SD card storage for data logging  
✔️ LDR for automatic head/rear light control  
✔️ XBee for wireless control and communication  
✔️ Arduino Mega as the main controller  

## 🔹 Hardware Components  
- **Microcontroller:** Arduino Mega  
- **Sensors:** UV sensor, Ultrasonic, GPS (with RTC), LDR  
- **Storage:** SD card module  
- **Communication:** XBee module  
- **Chassis:** 3-D printed and lasercut 

## 🔹 Software & Tools  
- **Programming:** C++ (Arduino)  
- **Data Logging:** SD card storage  
- **Communication Protocol:** XBee wireless @ 9600 baud rate
- **Navigation:** GPS-based tracking  
- **Simulation:** Proteus

## 🔹 How It Works  
1. The rover controlled using laptop or else with handheld 4 button pcb joystick by using X-Bee.  
2. The GPS logs the rover’s position with a date and timestamp.  
3. The ultrasonic sensors detect obstacles to avoid collisions.  
4. Data (UV levels, location, time, etc) is stored on an SD card.  
5. The LDR automatically turns the light on/off based on ambient brightness.  
6. Wireless transmission via XBee allows real-time monitoring.  

## 🔹 Installation & Setup  
📌 **Block Diagram** – Pictorial representation of overall connection with Arduino Mega
![image](https://github.com/user-attachments/assets/19947fb1-199f-4051-88e7-967c2ea9f9f3)

📌 **Circuit Diagram** - Including simulation
![image](https://github.com/user-attachments/assets/a29dd56c-4e15-4ecb-825a-b30097d992de)
![image](https://github.com/user-attachments/assets/ba7bf678-8bbd-4a01-a193-a52aefdb0b0b)

📌 **PCB Design**
![image](https://github.com/user-attachments/assets/b530c92b-e9df-4510-827c-8c49d4f648c0)
![image](https://github.com/user-attachments/assets/b6f3f7ed-b0fe-4a75-ac98-d803648166fd)
![image](https://github.com/user-attachments/assets/ecf0e019-2ce3-46c2-aca1-8cbdebbc3fff)
![image](https://github.com/user-attachments/assets/d4839cee-3384-4683-ad98-2b849401c41c)
![image](https://github.com/user-attachments/assets/e674494c-3154-4ab5-ac43-586f82f399f1)


## 🔹Results  
- User End output
  ![Screenshot (222)](https://github.com/user-attachments/assets/bdbdaa9c-1413-4f73-97fe-dcbb582c3eba)
  
- Data logs from SD card which includes all the sensor data per second (txt files).
  ![image](https://github.com/user-attachments/assets/8301e661-d504-4bb8-8a25-b2998da8354c)

- Demo video showcasing the working system:
  https://github.com/user-attachments/assets/db62c52c-53ec-4206-8c66-9a5446f7413e


## 🔹 Future Improvements  
🔹 Replace UV sensor with an actual radiation sensor like GM-Tube and include wireless camera.  
🔹 Improve navigation with popular controls like PID, Fuzzy logic, and AI control.  
🔹 Upgrade communication for better range and reliability.  
