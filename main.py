import sys
import os
from PyQt6.QtWidgets import QApplication

import config
from hardware import SerialRobotController
from gui import RobotArmWindow

def main():
    os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--enable-gpu-rasterization --ignore-gpu-blocklist"

    # Initialize the new Teensy connection
    controller = SerialRobotController(port=config.COM_PORT)
    controller.connect()
        
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    # Pass the serial controller into the GUI
    win = RobotArmWindow(controller)
    win.show()
    
    # Cleanup on exit
    app.exec()
    controller.shutdown()
    sys.exit()

if __name__ == "__main__":
    main()