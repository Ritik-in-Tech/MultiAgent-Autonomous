import os
from crewai_tools import Tool
import rospy
from geometry_msgs.msg import Twist
import openai
import requests

class TurtleBotTool(Tool):
    def __init__(self, namespace):
        self.namespace = namespace
        self.vel_pub = rospy.Publisher(f'/{self.namespace}/cmd_vel', Twist, queue_size=10)
        
        # Load API keys
        self.openai_key = os.getenv("OPENAI_API_KEY")
        self.serper_key = os.getenv("SERPER_API_KEY")

    def move_turtlebot(self, command: str):
        vel_msg = Twist()

        # Based on the command, issue movement instructions to the TurtleBot
        if command == "move_forward":
            vel_msg.linear.x = 0.2
        elif command == "stop":
            vel_msg.linear.x = 0.0
        else:
            return "Unknown command"

        # Publish the velocity message to the TurtleBot
        self.vel_pub.publish(vel_msg)
        return f"Executed command {command} on {self.namespace}"

    def query_openai(self, prompt: str):
        if not self.openai_key:
            return "OpenAI API key is missing!"
        
        openai.api_key = self.openai_key
        response = openai.Completion.create(
            model="text-davinci-003",
            prompt=prompt,
            max_tokens=150
        )
        return response.choices[0].text.strip()

    def query_serper(self, search_query: str):
        if not self.serper_key:
            return "Serper API key is missing!"
        
        headers = {
            'X-API-KEY': self.serper_key
        }
        params = {
            'q': search_query
        }
        response = requests.get('https://google.serper.dev/search', headers=headers, params=params)
        if response.status_code == 200:
            return response.json()
        else:
            return f"Error: {response.status_code} - {response.text}"
