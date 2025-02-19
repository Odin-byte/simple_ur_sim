import socket


class FtnAxia:
    """
    Class for reading SCHUNK FTN-AXIA80 ForceTorque Sensor over TCP.
    Use this inside a ROS-node.
    Commands have size of 20 bytes.
    ip: the ip address of the FT-Sensor. Default is 192.168.1.1
    port: the port where the FT-Sensor is listening. Default is 49151
    """
    READFT = 0
    READCALINFO = 1
    WRITETRANSFORM = 2
    WRITETHRESHOLD = 3

    WRITETRANSFORM_RESPONSE = 0x12340200
    WRITETHRESHOLD_RESPONSE = 0x12340300

    HEADER = 0x1234

    def __init__(self):
        self.ip = "127.0.0.1"
        self.port = 49151

        self.init_connection()
        print(self.read_calibration_info())

        self.Wrench = []

    def init_connection(self):
        """
        initializes a connection with the sensor over tcp using given ip and port.
        :return:
        """
        #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.s:
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ip, self.port))

    def read_ft(self) -> bool:
        """
        Reads the current force and torque values from the sensor.
        Values are stored in member variables, e.g. self.Fx_cal
        uint8 command;          // muss immer READFT (0) sein
        uint8 reserved[15];     // sollte immer auf Wert=0 sein
        uint16 MCEnable;        // Bitmap der zu aktivierenden MCs
        uint16 sysCommands;     // Bitmap der Systembefehle
        :return: True if success, False otherwise
        """
        cmd = self._config_command_read_ft()
        self.s.sendall(cmd)

        # returns bytes object
        recv_data = self.s.recv(1024)

        return self._process_read_ft_bytes(recv_data)

    def _config_command_read_ft(self) -> bytearray:
        cmd = bytearray(20)
        cmd[0] = self.READFT
        return cmd

    def _process_read_ft_bytes(self, recv_data) -> bool:
        if int.from_bytes(recv_data[0:2], "big") == self.HEADER:
            # Wert für F oder M = aktueller FT-Rohwert * Kalibrierungszählungen pro Einheit / 16-Bit Skalierungsfaktor
            self.Fx_raw = int.from_bytes(recv_data[4:6], "big", signed=True)
            self.Fy_raw = int.from_bytes(recv_data[6:8], "big", signed=True)
            self.Fz_raw = int.from_bytes(recv_data[8:10], "big", signed=True)
            self.Tx_raw = int.from_bytes(recv_data[10:12], "big", signed=True)
            self.Ty_raw = int.from_bytes(recv_data[12:14], "big", signed=True)
            self.Tz_raw = int.from_bytes(recv_data[14:16], "big", signed=True)

            self.Fx_cal = self.Fx_raw / self.countsPerForce * self.sf0
            self.Fy_cal = self.Fy_raw / self.countsPerForce * self.sf1
            self.Fz_cal = self.Fz_raw / self.countsPerForce * self.sf2
            self.Tx_cal = self.Tx_raw / self.countsPerTorque * self.sf3
            self.Ty_cal = self.Ty_raw / self.countsPerTorque * self.sf4
            self.Tz_cal = self.Tz_raw / self.countsPerTorque * self.sf5
            self.Wrench = [self.Fx_cal, self.Fy_cal, self.Fz_cal, self.Tx_cal, self.Ty_cal, self.Tz_cal]

            print(self.Wrench)
            return True

        else:
            # header is not 0x1234; something went wrong
            return False

    def read_calibration_info(self) -> bool:
        """
        reads the calibration info of the sensor.
        This includes units, counts, scaling factors.
        Units correspond to the following numbers:
        Units (Force, Torque):
        1: Pound, Pound-Inch
        2: Newton, Pound-foot
        3: Kilo-Pfund, Newtonmeter
        4: Kilo-Newton, Newtonmillimeter
        5: Kilogramm, Kilogrammzentimeter
        6. Gramm, Kilonewtonmeter

        :return: True if success, False otherwise
        """
        cmd = self._config_command_read_calibration_info()
        self.s.sendall(cmd)
        recv_data = self.s.recv(1024)

        return self._process_read_calibration_info_bytes(recv_data)

    def _config_command_read_calibration_info(self) -> bytearray:
        cmd = bytearray(20)
        cmd[0] = self.READCALINFO
        return cmd

    def _process_read_calibration_info_bytes(self, recv_data: bytes) -> bool:
        """
        reads the received data and stores values in member variables
        :param recv_data: received bytes from sensor response
        :return: True if success, False otherwise
        """
        if int.from_bytes(recv_data[0:2], "big") == self.HEADER:
            self.forceUnits = recv_data[2]
            self.torqueUnits = recv_data[3]
            self.countsPerForce = int.from_bytes(recv_data[4:8], "big", signed=True)
            self.countsPerTorque = int.from_bytes(recv_data[8:12], "big", signed=True)
            self.sf0 = int.from_bytes(recv_data[12:14], "big", signed=True)
            self.sf1 = int.from_bytes(recv_data[14:16], "big", signed=True)
            self.sf2 = int.from_bytes(recv_data[16:18], "big", signed=True)
            self.sf3 = int.from_bytes(recv_data[18:20], "big", signed=True)
            self.sf4 = int.from_bytes(recv_data[20:22], "big", signed=True)
            self.sf5 = int.from_bytes(recv_data[22:24], "big", signed=True)
            self.scalingFactors = [self.sf0, self.sf1, self.sf2, self.sf3, self.sf4, self.sf5]

            print(self.forceUnits)
            print(self.countsPerForce)
            print(recv_data[12:14])
            print(self.scalingFactors)
            return True

        else:
            return False

    def write_transform(self, dunits, runits, dx, dy, dz, rx, ry, rz) -> bool:
        """
        uint8 command;              // muss immer WRITETRANSFORM (2) sein
        uint8 transformDistUnits;   // Einheiten für dx, dy, dz
        uint8 transformAngleUnits;  // Einheiten für rx, ry, rz
        int16 transform[6];         // dx, dy, dz, rx, ry, rz
        uint8 reserved[5];          // sollte immer 0 sein

        * Die "transform"-Elemente werden mit 100 multipliziert, um
        eine gute Granularität mit ganzzahligen Werten zu
        erreichen.

        Code für Einheit: Krafteinheit, Drehmomenteinheit
        1: Inch, Grad
        2: Foot, Radiant
        3: Millimeter
        4: Zentimeter
        5: Meter
        :return: True if success, False otherwise
        """
        cmd = self._config_command_write_transform(dunits=dunits,
                                                   runits=runits,
                                                   dx=dx, dy=dy, dz=dz,
                                                   rx=rx, ry=ry, rz=rz)
        self.s.sendall(cmd)

        recv_data = self.s.recv(1024)
        if int.from_bytes(recv_data, "big") == self.WRITETRANSFORM_RESPONSE:
            return True
        else:
            return False

    def _config_command_write_transform(self, dunits, runits, dx, dy, dz, rx, ry, rz) -> bytearray:
        """
        configures the 20 byte command. Byte positions are zero-based.
        Bytes not filled are zero, e.g. 0x00
        :param dunits: one byte at position 1
        :param runits: one byte at position 2
        :param dx: two bytes at position 3 and 4
        :param dy: two bytes at position 5 and 6
        :param dz: two bytes at position 7 and 8
        :param rx: two bytes at position 9 and 10
        :param ry: two bytes at position 11 and 12
        :param rz: two bytes at position 13 and 14
        :return: the write command to be sent to the sensor
        """
        cmd = bytearray(20)
        cmd[0] = self.WRITETRANSFORM
        cmd[1] = dunits
        cmd[2] = runits
        cmd[3:5] = int.to_bytes(dx * 100, 2, byteorder="big", signed=True)
        cmd[5:7] = int.to_bytes(dy * 100, 2, byteorder="big", signed=True)
        cmd[7:9] = int.to_bytes(dz * 100, 2, byteorder="big", signed=True)
        cmd[9:11] = int.to_bytes(rx * 100, 2, byteorder="big", signed=True)
        cmd[11:13] = int.to_bytes(ry * 100, 2, byteorder="big", signed=True)
        cmd[13:15] = int.to_bytes(rz * 100, 2, byteorder="big", signed=True)
        return cmd

    def write_threshold(self, index, axis, outputCode, comparison, compareValue) -> bool:
        """
        uint8 command;      // muss immer WRITETHRESHOLD sein
        uint8 index;        // Index für das Überwachen der Bedingungen: 0-31
        uint8 axis;         // 0=fx, 1=fy, 2=fz, 3=tx, 4=ty, 5=tz
        uint8 outputCode;   // Ausgabe-Code der Bedingungsüberwachung
        int8 comparison;    // Code für Vergleich, 1 für "größer als" (>), -1 (0xFF) für "kleiner als" (<)
        int16 compareValue;  // Wert für Vergleich, geteilt durch 16 Bit, Skalierungsfaktor
        uint8 reserved[13]  // sollte immer 0 sein
        :return: True if success, False otherwise
        """

        cmd = self._config_command_write_threshold(index=index,
                                                   axis=axis,
                                                   outputCode=outputCode,
                                                   comparison=comparison,
                                                   compareValue=compareValue)
        self.s.sendall(cmd)

        recv_data = self.s.recv(1024)
        if int.from_bytes(recv_data, "big") == self.WRITETHRESHOLD_RESPONSE:
            return True
        else:
            return False

    def _config_command_write_threshold(self, index, axis, outputCode, comparison, compareValue) -> bytearray:
        """
        configures the 20 byte command. Byte positions are zero-    print(my_sensor.cmd)
        :param outputCode:  one byte position 3
        :param comparison: one byte position 4
        :param compareValue: two bytes at position 5 and 6
        :return: the write command to be sent to the sensor
        """
        cmd = bytearray(20)
        cmd[0] = self.WRITETHRESHOLD
        cmd[1] = index
        cmd[2] = axis
        cmd[3] = outputCode
        cmd[4] = int.from_bytes(int.to_bytes(comparison, 1, "big", signed=True), "big", signed=False)
        cmd[5:7] = int.to_bytes(compareValue // self.scalingFactors[axis], 2, "big", signed=True)
        return cmd


if __name__ == "__main__":
    my_sensor = FtnAxia()
    my_sensor.read_ft()
    # while(True):
    #     my_sensor.read_ft()
    #     print(my_sensor.Fz_cal)
