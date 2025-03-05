# 📡 Embedded Spatial Mapping System

## 🔍 Overview
The **Embedded Spatial Mapping System** is a microcontroller-based solution that utilizes a **Time-of-Flight (ToF) sensor** to capture spatial measurements and visualize 3D environments. This project was developed to create an affordable and efficient alternative to commercial LIDAR systems for indoor navigation and exploration.

## ✨ Features
- **Real-time spatial data acquisition** using the VL53L1X Time-of-Flight sensor.
- **Stepper motor control** for 360° rotational scanning.
- **Microcontroller integration** with I2C, UART, and GPIO.
- **Embedded firmware development** in **C/C++**.
- **Data visualization and processing** using **Python**.
- **PC communication via UART** for graphical representation.

## 🛠️ Technologies Used
- **Microcontroller**: ESP32
- **Sensors**: VL53L1X (ToF Sensor)
- **Protocols**: I2C, UART, GPIO
- **Languages**: C/C++, Python
- **Development Tools**: Keil/IAR (for embedded), VS Code, Python (for visualization)

## 📦 Project Structure
```
📁 Embedded-Spatial-Mapping
├── 📂 Keil_rajana8    # Embedded C/C++ firmware for microcontroller
├── 📂 PC_rajana8      # Python scripts for data visualization
├── 📂 docs            # Project documentation
└── README.md         # Project overview
```

## 🚀 Getting Started
### 1️⃣ Hardware Setup
1. Connect the VL53L1X ToF sensor to the microcontroller via **I2C**.
2. Interface a **stepper motor** to rotate the sensor for 360° scanning.
3. Use a **push button** for data acquisition control.
4. Connect **status LEDs** to indicate measurement activity.

### 2️⃣ Firmware Installation
1. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/Embedded-Spatial-Mapping.git
   ```
2. Navigate to the firmware directory and compile the code:
   ```sh
   cd firmware
   make build  # or use Keil/IAR depending on your toolchain
   ```
3. Flash the firmware to the microcontroller using a debugger/programmer.

### 3️⃣ Data Visualization
1. Install Python dependencies:
   ```sh
   pip install -r requirements.txt
   ```
2. Run the visualization script:
   ```sh
   python visualize_data.py
   ```
3. Observe the spatial mapping output in the **graphical user interface (GUI)**.

## 📊 Sample Output
![![image](https://github.com/user-attachments/assets/0ae9ba6e-e4bb-4d64-92bf-61f4ecbb5932)]
This plot was reconstronsctructed from a simple, straight hallway.

---
Made with ❤️ by [Aaron Rajan]
