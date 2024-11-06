#!/usr/bin/env python3
import time
import rospy
import argparse
import json
from llama_cpp import Llama
from langchain.llms.base import LLM
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from typing import Optional, List, Mapping, Any
from std_msgs.msg import String

class LlamaLLM(LLM):
    model_path: str
    llm: Llama

    @property
    def _llm_type(self) -> str:
        return "llama-cpp-python"

    def __init__(self, model_path: str, **kwargs: Any):
        model_path = model_path
        llm = Llama(model_path=model_path)
        super().__init__(model_path=model_path, llm=llm, **kwargs)

    def _call(self, prompt: str, stop: Optional[List[str]] = None) -> str:
        response = self.llm(prompt, stop=stop or [])
        return response["choices"][0]["text"]

    @property
    def _identifying_params(self) -> Mapping[str, Any]:
        return {"model_path": self.model_path}


    
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
    
def infer(str_input):

    topic1 = '/target_location'
    topic2 = '/obj_class'
    topic3 = '/action'

    rospy.init_node('llm_node',anonymous=True);
    pub1 = rospy.Publisher(topic1, String, queue_size=10)
    pub2 = rospy.Publisher(topic2, String, queue_size=10)
    pub3 = rospy.Publisher(topic3, String, queue_size=10)

    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--model", type=str, default="./mistral-7b-instruct-v0.2.Q4_K_M.gguf") #gemma-2b.Q4_K_M  ./llama-2-13b-chat.Q4_K_M.gguf
    args = parser.parse_args()

    # Load the model
    llm = LlamaLLM(model_path=args.model)

    
    # Using in a chain
   ##    input_variables=["product_query"],
    #    template='''\n\n### Instruction:\nGiven a informations "{guide line}", extract the following information in JSON format: 
   #     "Targetlocation": [<The place to go>], "Action ": [<After reach target location then what is doing or none>], "Object": [<An object, device, person, or animal is located at the target location >]. Provide only the json output"\n\n### Response:\n''', ##The object or the person in the location
    #)
    
    # prompt = PromptTemplate(
    # input_variables=["instruction"],
    # template='''\n\n### Instruction:\nGiven the instruction "{instruction}", extract the following information in JSON format: 
    # "TargetLocation": [<The place to go>], "Action": [<What action to take at the target location, if just observing and not any action fill the 'null'>], "Object": [<The object, device, person, or animal located at the target location or not object return null>].Provide only the json output and no need more instruction.\n\n### Response:\n'''
    # )
   
    # prompt = PromptTemplate(
    # input_variables=["query"],
    # template='''\n\n### Instruction:\nGiven the instruction "{instruction}", extract the following information in JSON format: 
    # "TargetLocation": [<The place to go>], "Action": [<What action to take at the target location otherwise the work to be done after the robot reach the destination, if just observing and not taking any action, use 'null'>], "Object": [<The object in the target location realted to the work to be done. it can be person, device, animal or any object, or 'null' if no object is specified>].
    # So example input query and model output are bellow.please refere these example and give the only model output is like following outputs and these output give the json file.

    # input: "Please go to the coffee room and see if the robot is there", Output: "TargetLocation": ["coffee room"], "Action": ["see if the robot is there"], "Object": ["robot"].
    
    # input: "Please go to the coffee room", Output:  "TargetLocation": ["coffee room"], "Action": ["null"], "Object": ["null"].

    # input: "Please go to the coffee room and put the trash in bin", Output: "TargetLocation": ["coffee room"], "Action": ["put the trash in bin"], "Object": ["bin"].

    # input: "Can you go to the professor room and check if the professor is there",Output: "TargetLocation": ["professor room"], "Action": ["check if the professor is there"], "Object": ["professor"] .
     
    # only output and no need more response
    #  \n\n### Response:\n'''
    # )
    

    # prompt = PromptTemplate(
    # input_variables=["query", "instruction"],
    # template='''\n\n### Instruction:\nFollowing the instruction "{instruction}", extract the specified information from the query in JSON format with the fields:
    # - "TargetLocation": The destination or place to go (e.g., meeting room, robotics lab, professor room, coffee room).
    # - "Action": The action to perform at the target location (e.g., make coffee, check if the professor is there). If no action is required, set to "null".
    # - "Object": The object associated with the action at the location (e.g., person, device, animal, or any specific item). If no object is mentioned, set to "null".

    # Refer to the examples below for guidance, and output only the JSON object in the required format.

    # Examples:
    # - input: "Please go to the coffee room and see if the robot is there"
    #   output: {{"TargetLocation": ["coffee room"], "Action": ["see if the robot is there"], "Object": ["robot"]}}

    # - input: "Please go to the coffee room"
    #   output: {{"TargetLocation": ["coffee room"], "Action": ["null"], "Object": ["null"]}}

    # - input: "Please go to the coffee room and put the trash in bin"
    #   output: {{"TargetLocation": ["coffee room"], "Action": ["put the trash in bin"], "Object": ["bin"]}}

    # - input: "Can you go to the professor room and check if the professor is there"
    #   output: {{"TargetLocation": ["professor room"], "Action": ["check if the professor is there"], "Object": ["professor"]}}

    # - input: "Please make me coffee"
    #   output: {{"TargetLocation": ["coffee room"], "Action": ["make coffee"], "Object": ["coffee machine"]}}
    # Output only the JSON object in the required format with no additional text.
    # \n\n### Response:\n'''
    # )
    
#     prompt = PromptTemplate(
#     input_variables=["query"],
#     template='''\n\n### Instruction:\n
#     Given the following query, output the information in a JSON format with the keys: "TargetLocation", "Action", and "Object".

#     Query: "{query}"

#     The JSON output should follow this structure:
#     - "TargetLocation": Where the action is directed (e.g., room name, location).
#     - "Action": What to do at that location (e.g., meet the professor, check the robot).
#     - "Object": The item or person involved (e.g., professor, robot).

#     Respond only with the JSON object. Do not include any additional text.

#     Example outputs:
#     - For "Please meet the professor", output: {{"TargetLocation": "professor room", "Action": "meet the professor", "Object": "professor"}}
#     - For "Go to the coffee room and check if the robot is there", output: {{"TargetLocation": "coffee room", "Action": "check the robot", "Object": "robot"}}

#     \n\n###response:\n
#     '''
# )

    ## Instruction:
    prompt = PromptTemplate(
    input_variables=["query"],
    template='''\n\n### Instruction:\nUsing "{query}" as a guide, extract the following from the query in JSON format for doing this task. these extract information send to robot complete the task: 
    So this area has 4 locations, meeting room, robotics lab, professor room and coffee room.  These location acociate object in the location. coffee room include bin,coffee machine , robotics room has robot and professor room in the professor and meeting room has Tv.
    - "TargetLocation": The location to go(e.g., meeting room, robotics lab, professor room, coffee room). If unstated, infer:
        - "make coffee" → "coffee room"
        - "meet professor" → "professor room"
        - "fetch equipment" → "robotics lab"
        - "attend meeting" → "meeting room"
    you find the object firstly in the room object
    - "Action": The task at the location. If none, set to "null".
    - "Object": The object associated with the action at the location (e.g., person, device, animal, or any specific item). If no object is mentioned, set to "null":
        - "make coffee" → "coffee machine","bin"
        - "meet professor" → "professor"
        - "fetch equipment" → "robot"
        - "attend meeting" → "TV"

    Output only the JSON object in this format.

    Examples:
    - "Please go to the coffee room and see if the robot is there" → {{"TargetLocation": ["coffee room"], "Action": ["see if the robot is there"], "Object": ["robot"]}}
    
    - "Go fetch the equipment" ,output: {{"TargetLocation": ["robotics lab"], "Action": ["fetch equipment"], "Object": ["equipment"]}}

    Output only the JSON format in the must required format include targetlocation, action,object with no additional text use only my query.\n\n### Response:\n
    '''
    )

#     prompt = PromptTemplate(
#     input_variables=["query"],
#     template='''\n\n### Instruction:\nUsing "{query}" as a guide, extract the following details in JSON format to send to the robot for completing the task. The area has the following locations and associated objects:
#     - coffee room: includes "coffee machine" and "bin"
#     - robotics lab: includes "robot"
#     - meeting room: includes "TV"
#     - professor's room: includes "professor"

#     Your output should contain:
#     - "TargetLocation": The location to go (e.g., coffee room, robotics lab, professor's room, meeting room). If unstated, infer based on keywords in the query:
#         - "make coffee" → "coffee room"
#         - "meet professor" → "professor's room"
#         - "fetch equipment" → "robotics lab"
#         - "attend meeting" → "meeting room"
#     - "Action": The task to be performed at the location. If not specified, set to "null".
#     - "Object": The object associated with the action at the location (e.g., "professor", "TV", "robot", "coffee machine", "bin").give me action relevent object atlocation. If no specific object is mentioned, must set to "null".

#     Example Outputs:
#     - Query: "Please go to the coffee room and refill coffee machine"
#       Output: {{"TargetLocation": ["coffee room"], "Action": ["refill coffee machine"], "Object": ["coffee machine"]}}

#     - Query: "Go fetch the equipment"
#       Output: {{"TargetLocation": ["robotics lab"], "Action": ["fetch equipment"], "Object": ["robot"]}}

      
#      - Query: "Please set up a presentation in the meeting room"
#       Output: {{"TargetLocation": ["meeting room"], "Action": ["set up a presentation"], "Object": ["TV"]}}

#     Output only the JSON object in the required format,must including TargetLocation, Action, and Object, with no additional text. Use only the query provided.\n\n### Response:\n
#     '''
# )



    # prompt = PromptTemplate(
    #     input_variables=["query"],
    #     template='''\n\n### Instruction:\nUsing "{query}" as a guide, extract the following details in JSON format to send to the robot for completing the task. The area has the following locations and associated objects:
    #     - coffee room: includes "coffee machine" and "bin"
    #     - robotics lab: includes "robot"
    #     - meeting room: includes "TV"
    #     - professor's room: includes "professor"

    #     Your output should contain:
    #     - "TargetLocation": The location to go (e.g., coffee room, robotics lab, professor's room, meeting room). If unstated, infer based on keywords in the query:
    #         - "make coffee" → "coffee room"
    #         - "meet professor" → "professor's room"
    #         - "fetch equipment" → "robotics lab"
    #         - "attend meeting" → "meeting room"
    #     - "Action": The task to be performed at the location. If not specified, set to "null".
    #     - "Object": The object associated with the action at the location (e.g., "professor", "TV", "robot", "coffee machine", "bin").you cant identifying object,set to "null" but try identifying the action related object at the location, .

    #     Example Outputs:
    #     - Query: "Please go to the coffee room and see if the robot is there"
    #     Output: {{"TargetLocation": ["coffee room"], "Action": ["see if the robot is there"], "Object": ["robot"]}}
        
    #     - Query: "Take this file to the professor at the professor's room"
    #     Output: {{"TargetLocation": ["professor's room"], "Action": ["take this file"], "Object": ["professor"]}}

    #     - Query: "Go fetch the equipment"
    #     Output: {{"TargetLocation": ["robotics lab"], "Action": ["fetch equipment"], "Object": ["robot"]}}

    #     Output only the JSON object in the required format, including TargetLocation, Action, and Object, with no additional text. Use only the query provided.\n\n### Response:\n
    #     '''
    # )
    
    strt_time = time.time()
# - "Please make me coffee" ,output:{{"TargetLocation": ["coffee room"], "Action": ["make coffee"], "Object": ["coffee machine"]}}
    chain = LLMChain(llm=llm, prompt=prompt)
    
    # Run the chain only specifying the input variable.
    user_query = "'''" + str_input + "'''"
    
    # output = chain.run(user_query)
    output = chain.run({"query": user_query})# , "instruction": instruction})
    print(output)

    # is_valid = validate_model_output(output)

    print(output)
    end_time = time.time()
    print(end_time-strt_time)
    # print(is_valid)
    #print(output_data["TargetLocation"])
    '''
    location,act, object = extract_information(output)
    target_loc,objec = validation(location,object)

    
   

    if(target_loc is not None):

        print("---------------------")
        print(target_loc)
        print(objec)
        
        for i in range(10):
            pub1.publish(target_loc)
            pub2.publish(objec)
            pub3.publish(act)
    '''   
    return output

if __name__ == "__main__":
    
    str_input =   "Check whether the video feed works in the meeting room"
    infer(str_input)