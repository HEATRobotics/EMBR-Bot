class ThermalGPSLocatorTester:
    def __init__(self, config):
        pass

    def GPS_ROS_Listener(self):
        pass

    # Reference Hot Box in World: thermalTestArea.world
    # Change of plan: Make the BLob a non robot (robot) that publishes its GPS location
    # that way we can subscribe to it here and compare from the script.
    hot_blob = {"x":4, "y":0, "z":0.5}

    def blobCordinateTest(ROS_topic, hot_blob):
        pass

if __name__ == "__main__":
    main()