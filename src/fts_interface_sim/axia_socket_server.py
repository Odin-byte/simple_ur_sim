import socket
import threading
import logging


class FtnAxiaServer:
    """
    Class for emulating the TCP Server of a SCHUNK FTN-AXIA80 ForceTorque Sensor over TCP.
    This class is to be used in combination with a pybullet simulation.
    """

    READFT = 0
    READCALINFO = 1
    WRITETRANSFORM = 2
    WRITETHRESHOLD = 3

    WRITETRANSFORM_RESPONSE = 0x12340200
    WRITETHRESHOLD_RESPONSE = 0x12340300

    HEADER = 0x1234

    MAX_SIZE_16_BIT = 32767

    def __init__(self, host_ip="127.0.0.1", port=49151, direct_unit_read=True):
        """Initialize the server object"""
        self.host_ip = host_ip
        self.port = port

        self.direct_unit_read = direct_unit_read

        self.calibration_range = 0  # This could be switched between 0 and 1 in a real sensor, but not in this fake hardware

        self.force_range_limit_xy = 500  # max / min value for the messurable forces along the x and y axis in Newton
        self.force_range_limit_z = (
            900  # max / min value for the messurable forces along the z axis in Newton
        )
        self.torque_range_limit = 20  # max / min value for the messurable torques around all axis in Newtonmeter

        self.counts_per_unit_force = 1000000
        self.counts_per_unit_torque = 1000000

        # Dynamically calculate the scaling_factors to use the full range of the 16 bit integers

        # scaling_factor for x and y forces
        factor_force_xy = int(
            self.force_range_limit_xy
            * self.counts_per_unit_force
            / self.MAX_SIZE_16_BIT
        )
        # scaling factor for z forces
        factor_force_z = int(
            self.force_range_limit_z * self.counts_per_unit_force / self.MAX_SIZE_16_BIT
        )
        # scaling factor for torques
        factor_torque = int(
            self.torque_range_limit * self.counts_per_unit_torque / self.MAX_SIZE_16_BIT
        )

        self.scaling_factors = [
            factor_force_xy,
            factor_force_xy,
            factor_force_z,
            factor_torque,
            factor_torque,
            factor_torque,
        ]

        print(self.scaling_factors)

        self.forces = [0, 0, 0]
        self.torques = [0, 0, 0]
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)

    def run_server(self):
        """Run the TCP server to accept and process client connections."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )  # Avoid 'Address already in use' error
            s.bind((self.host_ip, self.port))
            s.listen()
            print(f"Server started at {self.host_ip}:{self.port}")

            while True:
                con, addr = s.accept()
                print(f"Connected by {addr}")
                with con:
                    while True:
                        data = con.recv(1024)
                        if not data:
                            break
                        # Decypher recieved cmd
                        answer = self.decypher_cmd(data)

                        if answer != None:
                            con.sendall(answer)

    def start(self):
        """Start the server in a separate thread."""
        self.server_thread.start()
        print("FTN-Axia Server running in background...")

    def stop(self):
        self.server_thread.join(timeout=1)
        logging.info("Server stopped")

    def set_values(self, sensor_data: tuple):
        """Unpack the given tuple containing a pybullet joint state and set the current forces and torques messured by the sensor.

        Args:
            sensor_data (tuple): Tuple containing 4 items. Position, Velocity, Reaction Forces and applied Joint Motor Torque
        """
        forces_and_torques = sensor_data[2]

        print(forces_and_torques)

        if len(forces_and_torques) != 6:
            raise IndexError("Force and Torque Vector needs to contain 6 items")

        for i in range(3):
            self.forces[i] = round(forces_and_torques[i]) * (
                self.counts_per_unit_force if self.direct_unit_read else 1
            )
            self.torques[i] = round(forces_and_torques[i + 3]) * (
                self.counts_per_unit_torque if self.direct_unit_read else 1
            )

        # Limit the measurable range and scale down correctly
        for i in range(3):
            self.forces[i] = max(
                -self.MAX_SIZE_16_BIT,
                min(self.forces[i] // self.scaling_factors[i], self.MAX_SIZE_16_BIT),
            )

        for i in range(3):
            self.torques[i] = max(
                -self.MAX_SIZE_16_BIT,
                min(
                    self.torques[i] // self.scaling_factors[i + 3], self.MAX_SIZE_16_BIT
                ),
            )

    def get_sensor_data(self) -> tuple:
        """Read the latest messurments stored within the simulated sensor class

        Returns:
            tuple: tuple containing two lists of 3 values each, representing the forces and torques along / around the x, y and z axis of the sensor
        """
        return [self.forces, self.torques]

    def read_calibration_info(self) -> bytearray:
        """Read the calibration info data from the simulated sensor and return a bytearray containing the requested information

        Returns:
            bytearray: Information about the calibration of the simulated ft sensor
        """
        answer = bytearray(24)

        answer[0:2] = self.HEADER.to_bytes(2, byteorder="big")

        answer[2] = 2  # Unit of force Newton
        answer[3] = 3  # Unit of torque Newtonmeter

        answer[4:8] = self.counts_per_unit_force.to_bytes(
            4, byteorder="big", signed=True
        )  # 1 000 000
        answer[8:12] = self.counts_per_unit_torque.to_bytes(
            4, byteorder="big", signed=True
        )

        # scaling factors
        for i in range(len(self.scaling_factors)):
            answer[12 + 2 * i : 14 + 2 * i] = self.scaling_factors[i].to_bytes(
                2, byteorder="big", signed=True
            )

        return answer

    def read_ft_data(self) -> bytearray:
        answer = bytearray(16)

        answer[0:2] = self.HEADER.to_bytes(2, byteorder="big")

        forces, torques = self.get_sensor_data()

        for i in range(len(forces)):
            answer[4 + i * 2 : 6 + i * 2] = forces[i].to_bytes(
                2, byteorder="big", signed=True
            )

        for i in range(len(torques)):
            answer[10 + i * 2 : 12 + i * 2] = torques[i].to_bytes(
                2, byteorder="big", signed=True
            )

        return answer

    def decypher_cmd(self, data: bytearray) -> bytearray:
        """_Read the recieved bytestring, get the requested information and return the correct response as a bytestring.

        Args:
            data (bytes): bytearray holding the recieved cmd from the socket client

        Returns:
            bytes: bytearray containing the information requested by the cmd. If the cmd was invalid returns None.
        """
        # Extract cmd value from bytestring
        cmd = data[0]

        print(cmd)

        try:
            # Match the cmd (switch case)
            if cmd == self.READCALINFO:
                return self.read_calibration_info()

            elif cmd == self.READFT:
                return self.read_ft_data()

            elif cmd == self.WRITETHRESHOLD:
                raise NotImplementedError("Not implemented for simulated sensor")

            else:
                raise ValueError()
        except NotImplementedError:
            logging.error(f"Command {cmd} not implemented.")
            return None
        except ValueError:
            logging.error(f"Unkown command {cmd}")


# Example of running the server in a separate thread
if __name__ == "__main__":
    server = FtnAxiaServer()
    server.start()

    # Simulating main script
    while True:
        pass  # Your main program logic here
