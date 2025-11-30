# Voice-Operated Throttle Control

A Raspberry Pi–based voice-controlled throttle system using Vosk Speech Recognition and an SG90 servo motor.  
This project showcases real-time command recognition (“increase”, “stop”, “mode two”, etc.) and physical actuation using PWM control.

---

## 🚀 Overview  
This system enables acceleration control through **voice commands**.  
The Raspberry Pi listens continuously through a bluetooth mic, recognizes commands using the **Vosk offline speech engine**, and adjusts a **servo-based throttle mechanism** accordingly.  
A hardware **kill switch** ensures instant emergency shutdown.

---

## 🎤 Features  
- **Offline Speech Recognition** (Vosk) — no internet required  
- **Grammar-optimized recognizer** for high accuracy  
- **SG90 Servo Control** via PWM  
- **External Throttle Simulation** override logic  
- **Kill Switch (Emergency Stop)** with hardware interrupt  
- **GPS Mock Logging** to simulate INS/GPS module data  
- **Safe-hold state** for emergency braking  
- **Threaded architecture** for smooth audio, servo, logging, and simulation

---

## 🛠️ Hardware Used  
- **Raspberry Pi 5**  
- **Bluetooth Mic**  
- **SG90 Servo Motor**  
- **Push-Button** (Kill switch)  
- **External 5V supply for the servo**  

---

## 📦 Software Dependencies  

Key frameworks:
- Vosk Speech Recognition  
- SoundDevice  
- RPi.GPIO  
- Threading, Queue (Python built-ins)

---

## 📁 File Explanation  
### `voice_throttle.py`  
The main program controlling:
- Speech recognition threads  
- Command parsing  
- Servo actuation  
- External throttle simulation  
- GPS mock logging  
- Kill switch interrupt handling  
- Watchdog safety timer  

---

## 🎮 How Voice Commands Work  
Recognized commands include:

| Command | Action |
|--------|--------|
| **increase** | +10% throttle |
| **decrease** | −10% throttle |
| **stop** | throttle = 0% |
| **mode one** | 30 km/h (36% throttle) |
| **mode two** | 50 km/h (60% throttle) |
| **mode three** | 70 km/h (84% throttle) |
| **kill** | emergency safe-hold |

The recognizer uses a **grammar JSON** so it only listens for approved commands, improving accuracy dramatically.

---

## 🔒 Safety  
The system includes:
- **Immediate hardware kill switch**
- **Software watchdog** (auto-returns to safe throttle if no commands are heard)
- **Safe-hold PWM state** (servo locks to zero throttle)

---

## 📄 License & Copyright

© 2025 Gokul Akash S.

**No open-source license applied. All rights reserved.**


- This repository is publicly visible **for viewing only**.
- No permission is granted to copy, modify, distribute, or reuse any part of this project without explicit written permission from the author.


---

## ⭐ Acknowledgments  
- Vosk API for offline speech recognition  
- Open-source community that made this possible
