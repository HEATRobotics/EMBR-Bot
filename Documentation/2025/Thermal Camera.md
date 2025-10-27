# **FLIR Lepton 3.1R Radiometric Camera Guide**

This guide covers using the FLIR Lepton 3.1R camera with a PureThermal 3 (PT3) breakout board. The Lepton 3.1R is a **radiometric** camera, meaning it can output a 2D array of actual temperature values, not just a colorized image.

This document focuses on the software stack required to access this 14-bit raw temperature data, primarily on Linux.

## **The Core Challenge: Radiometry (14-bit) vs. Webcam Video (8-bit)**

The PureThermal 3 board can expose the Lepton camera to your computer in two different modes over USB:

1. **Standard Webcam Mode (8-bit AGC):** This is the default mode. The camera outputs a standard 8-bit colorized video stream (using Automatic Gain Control, or AGC). This is what applications like **OpenCV**, Zoom, or Photo Booth see by default. This mode **does not** contain temperature data.  
2. **Radiometric Mode (14-bit T-Linear):** This special mode outputs the raw 14-bit sensor data (in a Y16 format). In this "T-Linear" mode, the value of each pixel maps directly to a temperature (in centiKelvin).

To get temperature data, we **must** send a special command to the PureThermal 3 via USB to switch it from standard webcam mode to radiometric mode. Standard webcam libraries (like OpenCV's default `VideoCapture`) do not know how to send this command.

This is why we need a software stack based on `libuvc`.

## **Recommended Python Stack (flirpy)**

For accessing temperature data in Python, `flirpy` is the most direct and recommended tool. It handles all the low-level complexity for you.

### **1\. `libuvc` (The C-Library / "Driver")**

* **What it is:** A low-level C library that enables direct communication with USB Video Class (UVC) devices like your PureThermal 3\.  
* **Why you need it:** It's the foundation that allows `flirpy` (via `pyuvc`) to send the custom commands to enable radiometric mode. It is a required system-level dependency.  
* **Installation:**  
  * **Ubuntu/Debian:** `sudo apt install libuvc-dev`  
  * **macOS:** `brew install libuvc`  
  * **Windows:** This is more complex; it's often easier to use the Windows-specific FLIR application for diagnostics.

### **2\. `pyuvc` (The Python Wrapper)**

* **What it is:** A Python library that "wraps" libuvc. It allows Python code to access the low-level controls of the UVC device.  
* **Why you need it:** `flirpy` uses this library to do the actual work. You don't interact with it directly, but `flirpy` will install it as a dependency.

### **3\. `flirpy` (The High-Level FLIR Library)**

* **What it is:** A high-level Python library specifically for FLIR Lepton cameras on PureThermal boards.  
* **Why you need it:** This is the tool for your job. It finds the camera, sends the command to enable radiometry, and grabs the 14-bit raw image. This image is a 2D NumPy array where values are in **centiKelvin**. You must convert this to Celsius manually.  
* **Installation:**  
  `pip install flirpy`

* **Example Python Usage:** 
```python 
import flirpy.camera.lepton  
import numpy as np

# flirpy.camera.lepton.Lepton() will auto-detect the PT3  
try:  
    with flirpy.camera.lepton.Lepton() as cam:  
        # Grab a single 2D numpy array of raw centiKelvin values  
        raw_image = cam.grab()

        if raw_image is None:  
            print("Failed to capture image.")  
        else:  
            print(f"Got image with shape {raw_image.shape}")

            # Convert the raw centiKelvin to Celsius  
            celsius_image = (raw_image.astype(np.float32) / 100.0) - 273.15

            print(f"Min temp: {np.min(celsius_image):.2f} C, Max temp: {np.max(celsius_image):.2f} C")

except Exception as e:  
    print(f"Error connecting to Lepton camera: {e}")  
    print("Make sure libuvc is installed and udev rules are set.")
```
### **Linux `udev` Rule (Mandatory for Linux)**

On Linux, you must give your user account permission to access the PureThermal device.

1. Run the following command to create a `udev` rule file. This rule finds the PureThermal board (Vendor ID `1e4e`, Product ID `0100`) and gives it the correct permissions.
  ```  
  sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"
  ```

2. Apply the new rule without rebooting:  
  ```
  sudo udevadm control --reload-rules  
  sudo udevadm trigger
  ```

3. Unplug and replug your PureThermal 3 camera.

## **Diagnostics and Testing**

Before you start coding, it's wise to confirm the camera is working.

* **`GetThermal` (Linux)**  
  * An excellent (though no longer actively supported) desktop app for viewing the radiometric stream. It's the best way to confirm `libuvc` and your `udev` rules are working.  
  * **Instructions:**  
    1. Ensure `libuvc` is installed (see above).  
    2. Download the `GetThermal-*.AppImage` from the [v0.1.4 release page](https://github.com/groupgets/GetThermal/releases/tag/v0.1.4).  
    3. Make it executable: `chmod +x GetThermal-*.AppImage`
    4. Run it: `./GetThermal-*.AppImage`  
    5. You should see a live stream with a temperature scale and a cursor that reads temperature.  
* **FLIR Lepton Application (Windows)**  
  * The official FLIR Windows app for viewing the live stream, changing settings, and diagnosing the camera.  
  * **Download:** Find it on the [FLIR Lepton product page](https://oem.flir.com/en-hk/products/lepton/?vertical=microcam&segment=oem&docPage=2#Downloads) under "Software & Firmware".

## **Other Libraries & Frameworks (Context)**

These are other tools you might encounter, but they are generally *not* what you want for accessing temperature data from your PT3.

* **opencv (cv2) (The General Vision Library)**  
  * **Limitation:** As explained in the "Core Challenge," OpenCV's cv2.VideoCapture(0) will only see the 8-bit standard webcam stream. It cannot access the 14-bit radiometric data without significant, complex modification. **Do not use opencv to get temperature data.**  
* **pylepton (The Direct-Sensor Library)**  
  * **Limitation:** This library is for connecting a Lepton sensor *directly* to a single-board computer's GPIO pins (using SPI and I2C).  
  * **This is not for your USB connection** via the PureThermal board.  
* **v4l2 / Video4Linux2 (The OS Framework)**  
  * This is the kernel-level framework in Linux that handles all video devices. `libuvc` and opencv use this in the background, but you don't interact with it.

## **Important Considerations**

* **Image Dewarping (Lepton 3.1R):**  
  * The Lepton 3.1R has a wide-angle lens (71° HFOV), which causes **significant 'fisheye' distortion** in the raw image.  
  * If you need accurate spatial measurements (e.g., mapping temperatures to specific locations), you **must** apply a dewarping/undistortion algorithm to the image you get from `flirpy`.  
  * See the official FLIR application note: [Image Dewarping for 3.1R](https://oem.flir.com/en-ca/learn/thermal-integration-made-easy/lepton-3.1r-dewarping-application-note/).  
* **FFC (Flat-Field Correction):**  
  * The camera will periodically perform a "Flat-Field Correction" (FFC) to recalibrate its sensor.  
  * When this happens, you will hear a "click" from the camera’s shutter, and the video stream will freeze for a moment. This is normal and essential for accurate temperature readings. `flirpy` and GetThermal handle this automatically.

## **References and Useful Links**

* **FLIR Lepton Page (Downloads):** https://oem.flir.com/en-hk/products/lepton/?vertical=microcam\&segment=oem\&docPage=2\#Downloads  
* **FLIR Windows Integration:** https://oem.flir.com/en-ca/developer/lepton-family/lepton-integration-with-windows/  
* **3.1R Dewarping App Note:** https://oem.flir.com/en-ca/learn/thermal-integration-made-easy/lepton-3.1r-dewarping-application-note/  
* **flirpy Repository:** https://www.google.com/search?q=https://github.com/groupgets/flirpy  
* **uvc-radiometry.py Example:** The [purethermal1-uvc-capture repo](https://github.com/groupgets/purethermal1-uvc-capture/tree/master/python) contains a uvc-radiometry.py script. This is a great example of how to do what `flirpy` does, but manually using pyuvc.