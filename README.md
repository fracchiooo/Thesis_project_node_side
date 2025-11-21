# Proof of Concept for a Digital Twin of an Ultrasonic Fermentation System - Microcontroller part

## Prerequisites

- **ESP-IDF Framework**: The ESP-IDF development framework must be installed on your system
- **ESP32 Microcontroller**: An ESP32 microcontroller with Wi-Fi module, connected to the computer via USB

## Hardware Connections

Connect the following components to the specified GPIO pins:

### Ultrasonic Transducer Control (40 kHz)
- **GPIO 1**: PWM burst (duty cycle) signal output
- **GPIO 2**: PWM direction (frequency of resonance) signal output

### Ultrasonic Transducer Control (20 kHz)
- **GPIO 3**: PWM direction (frequency of resonance) signal output
- **GPIO 4**: PWM burst (duty cycle) signal output

### Temperature Sensing
- **GPIO 26**: Data input from Dallas DS18B20 temperature probe

### Frequency Detection
- **ADC1 Channel 0**: Analog input from frequency sensor (piezoelectric disk, with driver circuit)

### Status Indicator
- **GPIO 35**: Integrated LED for connection status indication

**Note**: Ensure your microcontroller has an integrated or externally connected LED on GPIO 35. This LED provides visual feedback for Wi-Fi and MQTT broker disconnection events.

## Software Configuration

### 1. Navigate to Project Directory
```bash
cd ./esp_node
```

### 2. Configure ESP-IDF Settings

Launch the configuration menu:
```bash
idf.py menuconfig
```

Apply the following configuration changes:

#### a) Increase Main Task Stack Size
- Navigate to: `Component config` → `ESP System Settings` → `Main task stack size`
- Set value to: **15000**

#### b) Enable C++ Exception Handling
- Navigate to: `Compiler options` → `Enable C++ exceptions`
- Set to: **true**

#### c) Configure Partition Table for Larger Application
- Navigate to: `Partition Table` → `Partition Table`
- Select: **Single factory app (large), no OTA**

#### d) Configure Connection Parameters

**MQTT Configuration** (`Example Configuration`):
- Broker URL: Enter your Cloud broker URL
- Username: Your MQTT authentication username
- Password: Your MQTT authentication password

**Wi-Fi Configuration** (`WiFi Configuration`):
- SSID: Your Wi-Fi access point name
- Password: Your Wi-Fi password

### 3. MQTT Certificate Setup

- Download the CA certificate from your broker
- Rename the certificate to `emqxsl-ca.crt`
- Place the certificate in `./esp_node/components/mqtts/certs/`

## Build and Flash Instructions

Execute the following commands in sequence:

### 1. Initialize ESP-IDF Environment
```bash
get_idf
```

**Note**: This command enables ESP-IDF tools. The exact command may vary depending on your ESP-IDF installation method. Refer to the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) if needed.

### 2. Set Target Microcontroller
```bash
idf.py set-target 
```

Replace `<your_esp32_model>` with your specific ESP32 variant (e.g., `esp32`, `esp32s3`, `esp32c3`).

### 3. Build the Project
```bash
idf.py build
```

### 4. Flash to Microcontroller
```bash
idf.py -p `<port>` flash
```

Replace `<port>` with the serial port where your microcontroller is connected (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).

### 5. Monitor Serial Output (Optional)

To view real-time logs from the device:
```bash
idf.py -p `<port>` monitor
```

---

**Project**: Proof of Concept for a Digital Twin of an Ultrasonic Fermentation System  
**Institution**: Sapienza University of Rome