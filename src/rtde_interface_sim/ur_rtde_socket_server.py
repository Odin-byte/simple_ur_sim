import socket
import threading
import logging
import struct

# Setting up basic logging configuration for better error tracking
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")


class UrRtdeServer:
    """
    Class for emulating the TCP Server used by the UR RTDE interface.
    This class is to be used in combination with a pybullet simulation of a UR Robot.
    """

    def __init__(self, host_ip="127.0.0.1", port=30004, direct_unit_read=True):
        """Initialize the server object """
        self.host_ip = host_ip
        self.port = port
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)

        # Frequency in which the server needs to publish feedback to the client. The value is requested by the client within 
        # the setup outputs msg
        self.feedback_frequency = None
        self.feedback_thread = None
        
        # Set by the controller
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Read in from the simulation and sent to the controller as feedback
        self.actual_TCP_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.actual_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.output_int_register_0 = 0

        self.available_params = {
            "actual_TCP_pose" : "VECTOR6D",
            "actual_q" : "VECTOR6D",
            "actual_qd" : "VECTOR6D",
            "target_qd" : "VECTOR6D",
            "output_int_register_0" : "INT32",
            "input_double_register_0" : "DOUBLE",
            "input_double_register_1" : "DOUBLE",
            "input_double_register_2" : "DOUBLE",
            "input_double_register_3" : "DOUBLE",
            "input_double_register_4" : "DOUBLE",
            "input_double_register_5" : "DOUBLE",
            "input_int_register_0" : "INT32",
            "input_int_register_1" : "INT32"
        }

        self.param_bytesizes = {
            "VECTOR6D" : "d d d d d d", # 6 double values
            "INT32" : "i", # 1 integer of size 4x8 bits
            "DOUBLE" : "d"
        }

        self.control_registers = []
        self.control_values = []

        self.feedback_registers = []

        # Byte structure strings of control (inputs) and feedback (outputs)
        self.control_data_structure = None
        self.feedback_data_structure = None

    def run_server(self):
        """Run the TCP server to accept and process client connections."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
                )  # Avoid 'Address already in use' error
                s.bind((self.host_ip, self.port))
                s.listen()
                logging.info(f"Server started @ {self.host_ip}:{self.port}")
            except socket.error as e:
                logging.error(f"Socket error occured: {e}")

            while True:
                try:
                    self.con, self.addr = s.accept()
                    logging.info(f"Connected by {self.addr}")
                    with self.con:
                        while True:
                            data = self.con.recv(1024)
                            if not data:
                                break
                            # Decypher recieved cmd
                            msg_len, packet_type, payload = self.decode_rtde_message(data)
                            self.handle_rtde_payload(packet_type, payload, msg_len)
                except socket.error as e:
                    logging.error(f"Error in connection handling: {e}")
                    continue

    def start(self):
        """Start the server in a separate thread."""
        self.server_thread.start()
        print("RTDE Interface Server running in background...")

    def start_feedback_loop(self):
        print("Starting feedback loop!")
        if self.feedback_thread:
            self.feedback_thread.cancel()

        interval = 1.0 / self.feedback_frequency

        def send_feedback():
            self.send_feedback_msg()
            self.feedback_thread = threading.Timer(interval, send_feedback)
            self.feedback_thread.start()
        
        send_feedback()

    def send_feedback_msg(self):
        if self.con:
            try:
                self.send_feedback_data_msg()
            except OSError as e:
                print(f"Failed to send feedback: {e}")

    def stop(self):
        """Stop the threaded TCP Server
        """
        self.server_thread.join(timeout=1)
        logging.info("Server stopped")

    def set_robot_status(self, joint_positions: tuple, tcp_pose: tuple):
        """Copy the given simulation values into the RTDE interface object

        Args:
            joint_positions (tuple): Current joint positions of the simulated robot
            tcp_pose (tuple): Current TCP pose of the robot in the world frame of the pybullet simulation
        """
        return
    
    def decode_rtde_message(self, data):
        if len(data) < 3:
            logging.error("Received message with invalid length: less than 3 bytes")
            return None, None, None
        try:
            # Extract lenght of payload
            msg_length = struct.unpack(">H", data[:2])[0]

            # Extract packettype
            packet_type = struct.unpack("B", data[2:3])[0]

            # Extract payload
            payload = data[3:msg_length+2]
            return msg_length, packet_type, payload
        except struct.error as e:
            logging.error(f"Error unpacking RTDE message: {e}")
            return None, None, None
        
    def handle_request_protocol_version(self, payload):
        version = struct.unpack(">H", payload)[0]  # uint16 aus Nutzdaten extrahieren
        response = struct.pack(">H B B", 4, 0x56, 1)  # Länge, Typ, Version
        self.con.sendall(response)

    def handle_get_ur_control_version(self, payload):
        response = struct.pack(">H B I I I I", 19, 118, 3, 2, 19171, 42)
        self.con.sendall(response)
    
    def handle_control_package_setup_outputs(self, payload):
        self.feedback_frequency = (struct.unpack(">d", payload[:8])[0])

        params = payload[8:].decode(errors="ignore").strip().split(",")
        param_types = []
        output_recipe_id = 1
        for requested_param in params:
            logging.debug(f"Requsted param: {requested_param}")
            if requested_param in self.available_params.keys():
                param_types.append(self.available_params[requested_param])
                self.feedback_registers.append(requested_param)
                # self.feedback_values.append(None)
            else:
                param_types.append("NOT_FOUND")
                output_recipe_id = 0

        # Update the expected input msg format
        structure_string = ">"
        for type in param_types:
            structure_string += self.param_bytesizes[type] + " "
        self.feedback_data_structure = structure_string

        response_byte_string = ",".join(param_types).encode("utf-8")
        msg_len = len(response_byte_string) + 1 + 3
        msg_type = 79

        response = struct.pack(f">H B B {len(response_byte_string)}s", msg_len, msg_type, output_recipe_id, response_byte_string)
        self.con.sendall(response)

        self.start_feedback_loop()
    
    def handle_control_package_setup_inputs(self, payload):
        params = payload.decode(errors="ignore").strip().split(",")
        param_types = []
        output_recipe_id = 1
        for requested_param in params:
            if requested_param in self.available_params.keys():
                param_types.append(self.available_params[requested_param])
                self.control_registers.append(requested_param)
                self.control_values.append(None)
            else:
                param_types.append("NOT_FOUND")
                output_recipe_id = 0
        
        # Update the expected input msg format
        structure_string = ">"
        for type in param_types:
            structure_string += self.param_bytesizes[type] + " "
        self.control_data_structure = structure_string
        response_byte_string = ",".join(param_types).encode("utf-8")
        msg_len = len(response_byte_string) + 1 + 3
        msg_type = 73
        response = struct.pack(f">H B B {len(response_byte_string)}s", msg_len, msg_type, output_recipe_id, response_byte_string)
        self.con.sendall(response)
    
    def handle_control_package_start(self, payload):
        # Has no payload
        msg_len = 4
        msg_type = 83
        # This fake hardware controller always accepts -> 1
        response = struct.pack(f">H B B", msg_len, msg_type, 1)
        self.con.sendall(response)

    def handle_control_package_pause(self, payload):
        # Has no payload
        msg_len = 4
        msg_type = 80
        # This fake hardware controller always accepts -> 1
        response = struct.pack(f">H B B", msg_len, msg_type, 1)
        self.con.sendall(response)

    def send_feedback_data_msg(self):
        # Use the defined feedback structure string and populate the msg with the current values of the feedback registers
        try:
            list_feedback_values = []
            for requested_value in self.feedback_registers:
                value = getattr(self, requested_value, None)
                if isinstance(value, list):
                    list_feedback_values.extend(value)
                else:
                    list_feedback_values.append(value)
            # Generate the bytestring containing the feedback values
            feedback_value_bytes = struct.pack(self.feedback_data_structure, *list_feedback_values)
            msg_type = 85
            msg_len = len(feedback_value_bytes) + 4
            response = struct.pack(f">H B B", msg_len, msg_type, 1) + feedback_value_bytes
            self.con.sendall(response)
        
        except struct.error as e:
            logging.error(f"Error packing feedback data: {e}")
        except socket.error as e:
            logging.error(f"Error sending feedback data: {e}")

    def handle_data_package(self, payload):
        # Unpack the control data send by the controller based on the latest control byte structure string
        control_values = struct.unpack(self.control_data_structure, payload[1:])
        for i, value in enumerate(control_values):
            self.control_values[i] = value

        # Send feedback msg
        self.send_feedback_data_msg()

    def handle_rtde_payload(self, type, payload, msg_len):
        logging.debug(f"Handling packet type: {type}")
    
        # Check for correct length of payload
        if len(payload) > msg_len - 3:
            payload = payload[:msg_len - 3]
        elif len(payload) < msg_len - 3:
            logging.error("Received payload is too short!")
            return
        # Handle known package types
        if type == 86:
            return self.handle_request_protocol_version(payload)
        elif type == 118:
            return self.handle_get_ur_control_version(payload)
        elif type == 79:
            return self.handle_control_package_setup_outputs(payload)
        elif type == 73:
            return self.handle_control_package_setup_inputs(payload)
        elif type == 83:
            return self.handle_control_package_start(payload)
        elif type == 85:
            return self.handle_data_package(payload)
        else:
            logging.error(f"Unknown RTDE payload type received: {type}")

# Example of running the server in a separate thread
if __name__ == "__main__":
    server = UrRtdeServer()
    server.start()

    # Simulating main script
    while True:
        pass  # Your main program logic here
