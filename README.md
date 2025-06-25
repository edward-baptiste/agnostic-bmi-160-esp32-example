# BMI160 ESP32 Example (ESP-IDF)

This repository is an example demonstrating how to integrate the **platform-agnostic BMI160 driver** with an **ESP32** using **ESP-IDF**.

It includes:

- Minimal FreeRTOS-based application
- I²C bus initialization for the ESP32
- HAL-style function pointers to connect ESP-IDF I²C to the BMI160 driver

> **Note**: This repo contains both the example application and the driver itself. The reusable BMI160 driver is maintained separately and can be accessed here **include link**

---

## Getting Started

### 1. Clone this repository

```bash
git clone https://github.com/yourusername/agnostic-bmi-160-esp32-example.git
cd esp32-bmi160-example
```

### 2. Set Desired Pinout Configuration in main/main.c
```c
#define SCL_IO_PIN 32 
#define SDA_IO_PIN 33 
#define PORT_NUMBER I2C_NUM_0
#define BMI160_VDD_PIN 0
```

### 3. Build and Flash to a real ESP32 or QEMU for testing

#### Real ESP32 setup
Build ESP IDF Project
```bash
idf.py fullclean
idf.py build
```

Flash Project
```bash
idf.py flash monitor
```

#### QEMU setup
Install tools and dependencies
```bash
brew install libgcrypt glib pixman sdl2 libslirp
brew install git meson ninja
```

Clone Espressif QEMU fork
```bash
git clone https://github.com/espressif/qemu.git
cd qemu
git submodule update --init --recursive
```

Build QEMU
```bash
mkdir build
cd build

CFLAGS="-Wno-deprecated-declarations" \
../configure \
  --target-list=xtensa-softmmu \
  --enable-debug \
  --extra-cflags="-I/usr/local/include/slirp" \
  --extra-ldflags="-L/usr/local/lib"

make -j$(sysctl -n hw.ncpu)
sudo make install
```

Build ESP IDF project
```bash
idf.py fullclean
idf.py build
```

Create Flash Image
```bash
esptool.py --chip esp32 merge_bin -o build/flash_image.bin --flash_mode dio --flash_freq 40m --flash_size 4MB \
0x1000 build/bootloader/bootloader.bin \
0x8000 build/partition_table/partition-table.bin \
0x10000 build/agnostic-bmi-160-esp32-example.bin --fill-flash-size 4MB
```

Run Using QEMU
```bash
qemu-system-xtensa -nographic -machine esp32 -serial mon:stdio -drive file=build/flash_image.bin,if=mtd,format=raw
```
To stop the running QEMU, press “Ctrl + A” followed by X.