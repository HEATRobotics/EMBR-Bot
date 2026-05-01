# UART and pin mapping for rpi 3 model B

Since you are on a **Raspberry Pi 3 Model B+**, the hardware peripheral `uart2` does not exist. The Pi 3 series only has two UARTs: **UART0** (high performance) and **UART1** (mini-UART). On this model, Pins 27 and 28 (GPIO 0/1) are strictly reserved for the System I2C/ID EEPROM and cannot be used for MAVLink communication.

To get this working, you must shift to the standard UART pins.

### Step 1: Hardware Move (Mandatory)

You must move your RFD900X wires to the only pins on the Pi 3B+ capable of serial communication:

* **RFD Pin 7 (RX)** → **Pi Pin 8 (GPIO 14 / TX)**
* **RFD Pin 9 (TX)** → **Pi Pin 10 (GPIO 15 / RX)**
* **RFD Pin 1 (GND)** → **Pi Pin 6 (GND)**
* **RFD Pin 4 (5V)** → **Pi Pin 4 (5V)**

---

### Step 2: Clean up `config.txt`

We need to remove the "UART2" configuration and disable Bluetooth so the high-quality UART (UART0) is available on the pins you just connected.

1. Open the file: `sudo nano /boot/firmware/config.txt`
2. **Remove** these lines (since they don't work on Pi 3):
* `dtoverlay=uart2`


3. **Add/Verify** these lines are at the bottom:
```text
enable_uart=1
dtoverlay=disable-bt

```


4. Save and **Reboot**: `sudo reboot`

---
