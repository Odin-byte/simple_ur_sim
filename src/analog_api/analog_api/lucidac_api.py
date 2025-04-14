import json
import requests
import time
import sys

class LucidacAPI:
    def __init__(
        self, host, port, json_folder="/home/docker/ros_ws/src/analog_api/jsons/"
    ):
        self.host = host
        self.port = port

        self.json_folder = json_folder
        self.current_config = None
        self.current_initial_conditions = None

        self.url = f"http://{self.host}:{self.port}"
        self.max_retries = 30
        self.retry_interval = 2

        # Check for health of provided host
        print("Checking health of endpoint!")
        retries = 0
        while retries < self.max_retries:
            try:
                response = requests.get(f"{self.url}/health")
                if response.status_code == 200:
                    print("Endpoint healthy!")
                    break
            except requests.RequestException:
                pass
            print(
                f"No connection found. Trying again in {self.retry_interval} seconds"
            )
            time.sleep(self.retry_interval)
            retries += 1

        if retries == self.max_retries:
            print("API endpoint is not ready!")
            sys.exit(1)

    def adapt_json_template(
        self, json_name: str, given_params: dict, set_missing_values_to_one: bool = True
    )->dict:
        """This funktion loads in a template json ode and uses the provided input variables and initial conditions to adapt the template.

        Args:
            json_name (str): Name of the template json file
            given_params (dict): Dictionary storing all needed configurable params and initial conditions used in the template.
            set_missing_values_to_one (bool): If True: If a param needed by the requested template is missing in the corresponding dict, set it to 1.0. If False, raise an error.
        
        Returns:
            json_ode (dict): Dictionary holding the ODE information in JSON style.

        Raises:
            ValueError: Is raised if any of the needed parameters or initial conditions for the requested ODE template are not given AND the set_missing_values_to_one flag is set to False.
            TypeError: Is raised if any of the given parameters or initial conditions is not of type int or float.
        """
        try:
            with open(f"{self.json_folder + json_name}.template.json") as file:
                json_template = json.load(file)
                print(f"Got template: {json_template}")
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error reading template file {json_name}.template.json: {e}")

        # Get all needed parameters to adapt the given template (variables + initial conditions)
        needed_parameters = json_template["parameters"] + list(json_template["conditions"].keys())

        print(needed_parameters)

        # Check if all needed parameters are given
        try:
            missing_params = [
                needed_param
                for needed_param in needed_parameters
                if needed_param not in given_params
            ]

            if missing_params and not set_missing_values_to_one:
                raise ValueError(f"The following parameters are missing: {missing_params}")

            for missing_param in missing_params:
                given_params[missing_param] = 1.0
        except ValueError as e:
            print(f"Caught error {e} while checking the parameters needed for adapting the {json_name} template.")
        
        # Substitute the parameters within the functions
        try:
            ode_functions = json_template["equations"]
            for ode_name, ode_function in ode_functions.items():
                for key, val in given_params.items():
                    if isinstance(val, (int, float)):
                        ode_function = ode_function.replace(key, str(val))
                    else:
                        raise TypeError(f"Unsupported value for parameter '{key}': {val} (type: {type(val)})")
                # Reassign the ode function to the key
                ode_functions[ode_name] = ode_function
        except TypeError as e:
            print(f"Caught error {e} while adapting ODE function {ode_function}.")

        # Reassign the ode_functions dict back to the template dict
        json_template["equations"] = ode_functions

        # Set the correct initial conditions
        try:
            initial_conditions = json_template["conditions"]
            for ic_name, ice_value in initial_conditions.items():
                if isinstance(given_params[ic_name], (int, float)):
                    initial_conditions[ic_name] = given_params[ic_name]
                else:
                    raise TypeError(f"Unsupported value for initial condition '{ic_name}': {given_params[ic_name]} (type: {type(given_params[ic_name])})")
        except TypeError as e:
            print(f"Caught error {e} while adapting initial condition {ic_name}.")
        
        # Reassign the initial conditions back to the template dict
        json_template["conditions"] = initial_conditions

        #TODO: Check if this is needed
        # Clear parameter values in template
        json_template["parameters"] = []

        return json_template


    def load_ode_from_file(self, json_name: str) -> dict:
        """Load in a JSON file, describing an ODE.

        Args:
            json_name (string): Name of the JSON file without the file ending stored in the jsons directory of the ros pkg.

        Returns:
            json_ode (dict): Dictionary holding the ODE information in JSON style.

        Raises:
            FileNotFoundError: Gets raised if an invalid json file is given.
        """
        try:
            with open(f"{self.json_folder + json_name}.json") as file:
                json_ode = json.load(file)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error reading file {json}.json: {e}")
            sys.exit(1)

        return json_ode

    def compile_json_ode(self, json_dict_ode:dict):
        """Compile the given ODE based on the json dict passed.

        Args:
            json_dict_ode (dict): Dictionary with json like structure of an ODE.

        Raises:
            ValueError: Is raised when the config returned by the compiler is invalid.
        """
        ode_name = json_dict_ode["name"]

        # Extract the initial conditions from the json dict
        self.current_initial_conditions = json_dict_ode["conditions"]

        # Create the compile request
        request_data = {"backend": "lucidac", "data": json_dict_ode}

        # Send ODE to the compiler
        print(f"Sending compile request for {ode_name} ODE")
        try:
            response = requests.post(
                f"{self.url}/api/v0/compile",
                headers={"Content-Type": "application/json"},
                json=request_data,
            )
            response.raise_for_status()
            compile_response = response.json()

            # Extract config of the lucidac
            self.current_config = compile_response.get("config")

            # Check if the returned config is valid
            if not self.current_config:
                raise ValueError("No valid config was returned!")
        except (requests.RequestException, ValueError) as e:
            print(f"Caught error {e} while trying to compile {ode_name} ODE.")

    def calculate_current_ode(
        self, op_secs: float, sample_rate: int) -> list:
        """Adapt the current config stored in the handler based on the given variables and inicial conditions and send it
        to the API for calculation.

        Args:
            op_secs (float): Max time to calculate up to from now in seconds.
            sample_rate (int): Sample rate to read the returned values in.

        Returns:
            list: Calculated values returned by the API.
        """
        sim_request = {
            "config": self.current_config,
            "op_secs": op_secs,
            "sample_rate": sample_rate
        }

        print(f"Sending current ODE for calculation!")
        sim_response = requests.post(
                f"{self.url}/api/v0/simulate",
                headers={"Content-Type": "application/json"},
                json=sim_request
            )
        sim_response.raise_for_status()

        return json.dumps(sim_response.json(), indent=2)


if __name__ == "__main__":

    luci_api = LucidacAPI(host="localhost", port=5000)
    print(f"Testing compiler!")
    json_ode = luci_api.adapt_json_template("force_control", {"M_z": 7}, set_missing_values_to_one=True)
    luci_api.compile_json_ode(json_ode)
    print(f"Got config: {luci_api.current_config}")
    # luci_api.load_ode_from_file("force_control")
    # if luci_api.current_config != None:
    #     print(f"Got config: {luci_api.current_config}")
    #     luci_api.calculate_current_ode(0.2, 2000)
    # else:
    #     print("Test failed!")
