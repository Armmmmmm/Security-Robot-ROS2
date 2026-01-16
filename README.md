# Security Robot ROS2 Project 🤖

โปรเจกต์หุ่นยนต์รักษาความปลอดภัย (Security Robot) พัฒนาด้วย **ROS2** โดยใช้ **Jetson Nano** เป็นตัวประมวลผลหลัก และใช้ **ESP32** ผ่าน **micro-ROS** ในการควบคุมฮาร์ดแวร์ระดับล่าง (Low-level control) พร้อมติดตั้ง **RPLidar** สำหรับการทำแผนที่และเดินอัตโนมัติ

## 🛠️ ฮาร์ดแวร์ที่ใช้ (Hardware)
* **Main Computer:** NVIDIA Jetson Nano
* **Microcontroller:** ESP32 (ติดตั้ง micro-ROS)
* **Sensor:** RPLidar A1
* **Driver:** BTS7960
* **Power:** Battery 24VDC Li-Po

## ⚙️ การติดตั้ง (Installation)

1. **Clone Repository**
   ```bash
   cd ~/ros2_ws/src
   git clone [https://github.com/Armmmmmm/Security-Robot-ROS2.git](https://github.com/Armmmmmm/Security-Robot-ROS2.git)