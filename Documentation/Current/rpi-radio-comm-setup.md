# Connection Bridge from Windows to Linux
To get your RFD900X radio working inside WSL (Ubuntu 22.04), you need to "bridge" the USB connection from Windows to Linux. Windows owns the hardware by default, so `usbipd` acts as the middleman.

### 1. Install usbipd-win on Windows

First, ensure you have the software installed on your **Windows host** (not inside WSL).

1. Download the latest `.msi` installer from the [usbipd-win GitHub releases](https://github.com/dorssel/usbipd-win/releases).
2. Run the installer.
3. Open a **PowerShell** or **Command Prompt** with **Administrator privileges**.

---

### 2. Identify and Bind the Radio

You need to tell Windows that it's okay to let WSL "borrow" this specific USB device.

1. **List all USB devices:**
```powershell
usbipd list

```


Look for a device named something like "Silicon Labs CP210x USB to UART Bridge" or similar (the RFD900X uses a standard USB-to-Serial chip). Note the **BUSID** (e.g., `2-3`).
2. **Bind the device:**
This step is only required once. It prepares the device for sharing.
```powershell
usbipd bind --busid <BUSID>

```



---

### 3. Attach the Device to WSL

Now, you actually send the connection over to your Ubuntu distro.

1. **Attach the device:**
```powershell
usbipd attach --wsl --busid <BUSID>

```


*Note: Keep this terminal window open if you didn't use a persistent flag, though usually, it stays attached until unplugged or the WSL instance shuts down.*
2. **Verify in Ubuntu:**
Open your **WSL Ubuntu-22.04** terminal and run:
```bash
lsusb

```


You should see the Silicon Labs / RFD900X device listed there.

---

### 4. Locate the Port in WSL

In Linux, USB-to-Serial devices are mapped to `/dev/tty`. Usually, the first one plugged in is:
` /dev/ttyUSB0`

**Check permissions:**
By default, your user might not have permission to read/write to the serial port. Run this in WSL to grant access:

```bash
sudo chmod 666 /dev/ttyUSB0

```

---

### Common Gotchas

* **WSL Kernel:** If `lsusb` shows the device but your Python script says `/dev/ttyUSB0` doesn't exist, you might be on a very old WSL kernel. Update it by running `wsl --update` in PowerShell.
* **Automatic Detach:** If you unplug the RFD900X and plug it back in, you must run the `usbipd attach` command again.
* **Docker:** If you are running your code inside a **Docker container** within WSL, remember to start your container with the `--privileged` flag or by mapping the device specifically:
`docker run -it --privileged -v /dev/ttyUSB0:/dev/ttyUSB0 my-mavlink-image`

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
