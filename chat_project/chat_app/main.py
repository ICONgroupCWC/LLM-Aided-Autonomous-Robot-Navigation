import json

def extract(json_data):
    # Read the JSON file
    data = json.loads(json_data)

    
    # Extract specific entries
    location = data.get("Targetlocation")
    action = data.get("Action")
    object = data.get("Object")
    
    return location, action, object