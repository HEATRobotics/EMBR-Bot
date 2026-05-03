#!/bin/bash

# Start Services for Bluetooth and D-Bus
service bluetooth start
service dbus start  

# Start Application
npm run start