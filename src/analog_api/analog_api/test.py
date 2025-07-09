import json
import requests
import time
import sys

# Note:
# Start docker with `docker run -d --rm --network=host --name analog-test-container analog-container`

def main():
    # API connection parameters
    API_HOST = "localhost"
    API_PORT = 5000
    API_URL = f"http://{API_HOST}:{API_PORT}"
    MAX_RETRIES = 30
    RETRY_INTERVAL = 2
    
    # Wait for the API to be ready
    print("Waiting for API to be ready...")
    retries = 0
    while retries < MAX_RETRIES:
        try:
            response = requests.get(f"{API_URL}/health")
            if response.status_code == 200:
                print("API is ready!")
                break
        except requests.RequestException:
            pass
        
        print(f"API not ready yet. Retrying in {RETRY_INTERVAL} seconds...")
        time.sleep(RETRY_INTERVAL)
        retries += 1
    
    if retries == MAX_RETRIES:
        print("API didn't become ready in time. Exiting.")
        sys.exit(1)

    print("===================================================================")
    print("TESTING COMPILE > RUN ROUTE")
    print("===================================================================")
    
    # Read the sin.json file
    try:
        with open('../jsons/sin.json', 'r') as file:
            sin_json = json.load(file)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error reading sin.json: {e}")
        sys.exit(1)
    
    # Create the compile request
    request_data = {
        "backend": "lucidac",
        "data": sin_json
    }

    print(f"Request data: {request_data}")
    
    # Send compilation request
    print("Sending sin.json to API...")
    try:
        response = requests.post(
            f"{API_URL}/api/v0/compile",
            headers={"Content-Type": "application/json"},
            json=request_data
        )
        response.raise_for_status()
        compile_response = response.json()
        
        print("API Compile Response:")
        print(json.dumps(compile_response))
        
        # Extract config for simulation
        config = compile_response.get('config')
        
        # Check if we have a valid config
        if not config:
            print("Failed to get valid config from compile response. Skipping simulation.")
        else:
            print("Using extracted config for simulation...")
            
            # Create the simulation request with the extracted config
            sim_request = {
              "config": config,
              "op_secs": 0.1,
              "sample_rate": 100000,
              "sample_all": False,
              "k0": 1,
              "device": "lucidac"
            }
            
            # Send simulation request
            print("Sending simulation request to API...")
            sim_response = requests.post(
                f"{API_URL}/api/v0/run",
                headers={"Content-Type": "application/json"},
                json=sim_request
            )
            sim_response.raise_for_status()
            
            print("API Simulation Response:")
            print(json.dumps(sim_response.json(), indent=2))
            
    except requests.RequestException as e:
        print(f"API request failed: {e}")
        sys.exit(1)
    
    print("Test completed.")

    print("===================================================================")
    print("TESTING SIMULATE ROUTE")
    print("===================================================================")

    # Read the sin_ode.json file
    try:
        with open('../jsons/sin_ode.json', 'r') as file:
            sin_ode_json = json.load(file)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error reading sin_ode.json: {e}")
        sys.exit(1)
    
    # Create the simulate request
    request_data = {
        "ode": sin_ode_json,
        "op_secs": 0.2,
        "sample_rate": 100000,
        "sample_all": False,
        "k0": 10000
    }
    
    # Send compilation request
    print("Sending sin.json to API...")
    try:
        # Send simulation request
        print("Sending simulation request to API...")
        sim_response = requests.post(
            f"{API_URL}/api/v0/simulate",
            headers={"Content-Type": "application/json"},
            json=request_data
        )
        sim_response.raise_for_status()
        
        print("API Simulation Response:")
        print(json.dumps(sim_response.json(), indent=2))
        
    except requests.RequestException as e:
        print(f"API request failed: {e}")
        sys.exit(1)

    print("Test completed.")

if __name__ == "__main__":
    main()