from django.shortcuts import render
from django.http import JsonResponse
from .models import Message
import json
from .infer import infer
from .main import extract
from .pub import Pub_data
import rospy
from std_msgs.msg import String

topic1 = '/target_location'
topic2 = '/object_id'
topic3 = '/action'


pub1 = rospy.Publisher(topic1, String, queue_size=10)
pub2 = rospy.Publisher(topic2, String, queue_size=10)
pub3 = rospy.Publisher(topic3, String, queue_size=10)

def validate_model_output(output):
    try:
        # Try to load the output as JSON
        data = json.loads(output)

        # Check if all required keys are present
        required_keys = ["TargetLocation", "Action", "Object"]
        for key in required_keys:
            if key not in data:
                print(f"Missing key: {key}")
                return False
            if not isinstance(data[key], list):
                print(f"Key '{key}' is not a list.")
                return False

        print("The output is valid JSON and meets the required structure.")
        return True

    except json.JSONDecodeError:
        print("The output is not valid JSON.")
        return False

def index(request):
    return render(request, 'chat_app/index.html')

def send_message(request):
    
    if request.method == 'POST':
        data = json.loads(request.body)
        message_content = data['message']
        message = Message.objects.create(content=message_content)
        print(str({message.content}))
        out_ = infer(str({message.content}))
        flag = validate_model_output(out_)
        if(flag==True):
            target_loc,act,objec = extract(out_)
            print(target_loc)
            print(act)
            print(objec)
            rospy.init_node('llm_node', anonymous=True)
            # Pub_data(target_loc,act,objec)
            if(objec is None):
                objec = str(None)
            if(act is None):
                act = str(None)
            if(target_loc is not None):

                for i in range(10):
                    pub1.publish(target_loc)
                    pub2.publish(objec)
                    pub3.publish(act)
                result = "Target location: " +target_loc+  ",Action: " +act + ",Object: " + objec    #.format(target_loc, act, objec)
            #print(f"Received message: {message.content}")
                return JsonResponse({'message': f"{result}"})
            else:
                return JsonResponse({'message': "Please enter the command again."})

        else:
            return JsonResponse({'message': "Please enter the command again."})