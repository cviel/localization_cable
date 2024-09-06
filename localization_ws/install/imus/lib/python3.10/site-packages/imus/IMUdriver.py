import time
import numpy as np
from serial import Serial
import os

"""
Robot referential (R1)
XYZ : East-South-Down
"""


class IMU:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=57600, parameter_path="../cfg"): #old port='ttyUSB1' , old bauderate 115200
        self.serial = Serial(port, baud_rate, timeout=1)  # open serial port
        print('Serial connection initialized on port', self.serial.name)
        print('With baud rate',baud_rate)
        
        self.diplay_mode()
        self.euler_angles = [0.,0.,0.]
        self.mag = [0.,0.,0.] # for calibration only
        self.mag_updated = False # flag to detect new mag value

        #flush manually
        print("Flushing first 200 IMU entries...")
        for x in range(0, 200):
            line = bytearray(self.serial.readline())
            
        print("Loading parameters")
        self.get_parameters(parameter_path)
        print("Ready to go!\n")
        
    def get_parameters(self,parameter_path="../cfg"):
        try:
            # open the text files 
            
            print("Parameter path:", parameter_path)
            
            # f_center = open(parameter_path+"/magn_ellipsoid_center.txt","r")
            # f_transform = open(parameter_path+"/magn_ellipsoid_transform.txt","r")
            
            # magn_ellipsoid_center = f_center.readline().split()
            # magn_ellipsoid_transform = []
            # for i in range(3):
            #     magn_ellipsoid_transform.append(f_transform.readline().split())

            with open(os.path.join(parameter_path, "magn_ellipsoid_center.txt"), 'r') as f:
                magn_ellipsoid_center = f.readlines()
            
            with open(os.path.join(parameter_path, "magn_ellipsoid_transform.txt"), 'r') as f:
                magn_ellipsoid_transform = []
                for i in range(3):
                    magn_ellipsoid_transform.append(f.readline().split())

            print("Magnetic center :", magn_ellipsoid_center)
            print("Magnetic transform :", magn_ellipsoid_transform)
    
            print("Stop datastream")
            self.serial.write(('#o0').encode("utf-8")) # stop datastream
    
            # write the param in the IMU with the serial connection    
            # ~ my_imu.serial.write(('#cmxm' + str(-100)).encode("utf-8"))
            # ~ my_imu.serial.write(('#cmxM' + str(100)).encode("utf-8"))
            # ~ my_imu.serial.write(('#cmym' + str(-100)).encode("utf-8"))
            # ~ my_imu.serial.write(('#cmyM' + str(100)).encode("utf-8"))
            # ~ my_imu.serial.write(('#cmzm' + str(-100)).encode("utf-8"))
            # ~ my_imu.serial.write(('#cmzM' + str(100)).encode("utf-8"))

            self.serial.write(('#ccx' + magn_ellipsoid_center[0]).encode("utf-8"))
            self.serial.write(('#ccy' + magn_ellipsoid_center[1]).encode("utf-8"))
            self.serial.write(('#ccz' + magn_ellipsoid_center[2]).encode("utf-8"))
            self.serial.write(('#ctxX' +magn_ellipsoid_transform[0][0]).encode("utf-8"))
            self.serial.write(('#ctxY' +magn_ellipsoid_transform[0][1]).encode("utf-8"))
            self.serial.write(('#ctxZ' +magn_ellipsoid_transform[0][2]).encode("utf-8"))
            self.serial.write(('#ctyX' +magn_ellipsoid_transform[1][0]).encode("utf-8"))
            self.serial.write(('#ctyY' +magn_ellipsoid_transform[1][1]).encode("utf-8"))
            self.serial.write(('#ctyZ' +magn_ellipsoid_transform[1][2]).encode("utf-8"))
            self.serial.write(('#ctzX' + magn_ellipsoid_transform[2][0]).encode("utf-8"))
            self.serial.write(('#ctzY' + magn_ellipsoid_transform[2][1]).encode("utf-8"))
            self.serial.write(('#ctzZ' + magn_ellipsoid_transform[2][2]).encode("utf-8"))

            print("Resume datastream")
            self.serial.write(('#o1').encode("utf-8"))
            
            self.print_parameters()

        except FileNotFoundError:
            print("No parameter files found, make sure to run magnetometer_calibration nodes first.")
            if parameter_path != "##CALIBRATING##":
                exit()

        except Exception as e:
            print("Error loading the parameters")
            print(e)
        
    def diplay_mode(self):
        # Output angles in TEXT format 
        # Output frames have form like 
        # "#YPR=-142.28,-5.38,33.52"
        # followed by carriage return and line feed [\r\n]).
        self.serial.write(('#o1').encode("utf-8")) #  ENABLE continuous streaming output.
        self.serial.write(('#ot').encode("utf-8"))
        print("Display mode on")
    
    # ~ def calib_mode(self):
        # ~ # Output RAW SENSOR data of all 9 axes in TEXT format.
        # ~ # One frame consist of three lines - one for each sensor: 
        # ~ # acc, mag, gyr.
        # ~ self.serial.write(('#oc').encode("utf-8")) # Go to CALIBRATION output mode.
        # ~ self.serial.write(("#osrt").encode("utf-8")) # type of message str
        # ~ print("calibration mode on")
        
    def print_parameters(self):
        # print the parameters memorized by the IMU
        print("Stop datastream")
        self.serial.write(('#o0').encode("utf-8")) # stop datastream
    
        #print calibration values for verification by user
        self.serial.flushInput()
        self.serial.write(('#p').encode("utf-8"))
        calib_data = self.serial.readlines()
        calib_data_print = "Printing set calibration values:\r\n"
        for row in calib_data:
            line = bytearray(row).decode("utf-8")
            calib_data_print += line
        print(calib_data_print)
        print("Resume datastream")
        self.serial.write(('#o1').encode("utf-8"))

    def update_once(self):
        if self.serial.in_waiting == 0:
            print("No data available yet")
            return
        
        try:
            # Read all data from the serial feed
            data = self.serial.read_all().decode('utf-8')
            
            # Ensure we have at least one complete line
            if "\r" not in data:
                print("No complete lines received yet")
                return
            
            # Extract the last complete lines
            data = data[:data.rfind("\r")]
            lines = data[data.find("\n")+1:]
            lines = lines.split("\r\n")
            lines = lines[-3:]
        except Exception as e:
            print("Error reading from serial port or decoding data.")
            print("Exception:", e)
            return

        try:
            for line in lines:
                # Magnetic measurement for the calibration
                if '#M-' in line:
                    print("Mag line found:", line)
                    line = line.replace("#M-R=", "").replace("#M-C=", "").replace("\r\n", "")
                    words = line.split(",")
                    self.mag = [float(words[0]), float(words[1]), float(words[2])]
                    self.mag_updated = True

                elif "#A-" in line:
                    # Process accelerometer data if needed
                    pass

                elif "G-" in line:
                    # Process gyroscope data if needed
                    pass

                elif '#YPR=' in line:  # Yaw, pitch, roll info only (deg)
                    line = line.replace("#YPR=", "").replace("\r\n", "")
                    words = line.split(",")
                    self.euler_angles = [float(words[0]), float(words[1]), float(words[2])]
                else:
                    print('I do not understand this line:')
                    print(line)
        except Exception as e:
            print('Error decoding this line:')
            print(lines)
            print('Exception:', e)


    def update(self):
        while True:
            try:
                self.update_once()
            except KeyboardInterrupt:
                break

    def get_data(self, key):
        if key in self.data.keys():
            return self.data[key]
        else:
            print('Unknown key.')

    def reset(self):
        self.serial.close()


if __name__ == '__main__':
    # test the IMU reading
    my_imu = IMU()
    running = True
    while running:
        try:
            my_imu.update_once()
            print("euler_angles",my_imu.euler_angles)
        except KeyboardInterrupt:
            running = False


