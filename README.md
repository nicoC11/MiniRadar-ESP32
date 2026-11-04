# ESP32 Mini Flight Radar ✈️📡

A real-time flight tracking radar system based on **ESP32** and built with **PlatformIO**. The device retrieves nearby aircraft position and telemetry data via HTTP requests to aviation APIs, renders the information on a TFT display using the `TFT_eSPI` library, and sends automated notifications through a Telegram bot.



## 🚀 Features

* **Real-time flight monitoring:** Periodic HTTPS requests to the **airplanes.live** API, sending your location's geographic coordinates (latitude and longitude) to query and filter aircraft flying within a specified radius.
* **Physical button screen navigation:** Easily cycle through display modes (Radar View ➔ Nearest Aircraft Details ➔ Flight History ➔ Radar View) using a single push button connected to GPIO 25.
* **Nearest aircraft detail screen:** Dedicated view showing complete telemetry for the aircraft closest to your location at any given moment, including callsign, aircraft type/model, altitude, speed, heading, and calculated distance from your coordinates.
* **Flight history and memory persistence:** Continuous logging of the most frequent flights passing through your coverage radius. Uses the microcontroller's Non-Volatile Storage (NVS) via `Preferences.h` to permanently store aircraft counts and statistics, preserving data even after powering off or restarting the ESP32.
* **Efficient JSON parsing:** Optimized processing of API responses using the `ArduinoJson` (v7) library.
* **SPI graphical rendering:** Smooth visualization of telemetry, map, and interface on a TFT display (ST7789 / ST7735 driver) using the `TFT_eSPI` library.
* **Telegram notifications:** Automated alerts sent to your phone via an integrated Telegram bot (`WiFiClientSecure`), filtered to notify only when detecting heavy aircraft (such as Boeing 747, Airbus A380) or military flights within the specified radius.
* **Credential security:** Complete isolation of sensitive data (Wi-Fi network, keys, coordinates) inside a `secrets.h` file kept strictly out of Git.



## 🛠️ Hardware and Wiring

### Required Hardware

* **Microcontroller:** Standard ESP32 development board (ESP32 DevKit / ESP-WROOM-32, 30 or 38 pins).
* **Display:** 240x320 SPI TFT LCD (ST7789 / ST7735 driver).
* **Jumper Wires:** Female-to-Male jumper wires.

### Pinout Table

| Component / Pin | ESP32 Pin | Description |
| --- | --- | --- |
| **VCC (TFT)** | `3.3V` | Display power supply |
| **GND (TFT)** | `GND` | Common ground |
| **CS** | `GPIO 5` | Chip Select (SPI) |
| **RESET** | `GPIO 4` | Display Reset |
| **DC (A0)** | `GPIO 2` | Data / Command Control |
| **SDA (MOSI)** | `GPIO 23` | SPI Data Transmit |
| **SCK (SCL)** | `GPIO 18` | SPI Clock |
| **LED / BL** | `GPIO 13` *(or 3.3V)* | Backlight Control (PWM / ON-OFF) |
| **Button SIG** | `GPIO 25` | Push button input signal |
| **Button VCC** | `GPIO 33` | Button module power supply (3.3V) |



## ⚙️ Environment Setup and Libraries

The project uses **PlatformIO** for dependency management and hardware configuration. All required settings are pre-configured in the [`platformio.ini`](https://www.google.com/search?q=platformio.ini) file included in the repository:

* **Library Management (`lib_deps`):** PlatformIO automatically downloads `ArduinoJson` (v7) and `TFT_eSPI` during the initial build.
* **Display Configuration (`build_flags`):** SPI frequencies, display driver (ST7789), and pin mappings are defined via build flags in `platformio.ini`, eliminating the need to manually edit the `User_Setup.h` file inside the `TFT_eSPI` library.

> 💡 **Note:** If you want to change connection pins or adjust other parameters such as SPI bus frequency, you can do so directly by editing the lines under `build_flags` in `platformio.ini`.



## 🔑 Project Setup and Credentials

For security reasons, private credentials are not included in the repository.

1. **Clone the repository:**
```bash
git clone https://github.com/YOUR_USERNAME/MiniRadar-ESP32.git
cd MiniRadar-ESP32
```


2. **Create the secrets file:**
Copy the template file located in the `include/` directory:
```bash
cp include/secrets.example.h include/secrets.h
```


3. **Fill in the variables:**
Open `include/secrets.h` in your text editor and define your actual parameters:
```cpp
#ifndef SECRETS_H
#define SECRETS_H

// Wi-Fi Network
#define SECRET_SSID "YOUR_WIFI_SSID"
#define SECRET_PASS "YOUR_WIFI_PASSWORD"

// Telegram Bot API
#define SECRET_TELEGRAM_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"
#define SECRET_TELEGRAM_CHAT_ID "YOUR_CHAT_ID"

// Location Coordinates (Latitude / Longitude)
#define SECRET_LATITUDE 0.000000f
#define SECRET_LONGITUDE 0.000000f

#endif
```


4. **Build and Upload:**
Connect your ESP32 via USB and run in the VS Code / PlatformIO terminal:
```bash
pio run -t upload
```





## 📄 License

This project is licensed under the **MIT** License. See the [`LICENSE`](https://www.google.com/search?q=LICENSE) file for details.