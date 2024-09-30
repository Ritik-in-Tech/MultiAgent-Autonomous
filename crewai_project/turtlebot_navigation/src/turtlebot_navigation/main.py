import os
from dotenv import load_dotenv
load_dotenv()
from crewai import Agent, Task, Crew, Process
from tools.turtlebot_tool import TurtleBotTool

openai_key = os.getenv("OPENAI_API_KEY")
serper_key = os.getenv("SERPER_API_KEY")

def main():
    # === Define Agents ===
    
    # CI Agent (Campus Intelligence)
    ci_agent = Agent(
        role="Campus Intelligence Agent",
        goal="Navigate across the campus and guide other agents",
        verbose=True
    )

    # B1 Agent (Building 1 Agent)
    b1_agent = Agent(
        role="Building 1 Agent",
        goal="Move to a host location inside the building",
        verbose=True
    )

    # Visitor Agent (Follows the CI agent)
    visitor_agent = Agent(
        role="Visitor Agent",
        goal="Follow the CI agent to the host location",
        verbose=True
    )

    # === Assign TurtleBot Tools (custom ROS tools) ===
    
    # TurtleBotTool allows each agent to interact with its respective TurtleBot and query APIs
    turtlebot_tool_ci = TurtleBotTool(namespace="turtlebot1")  # CI agent's TurtleBot
    turtlebot_tool_b1 = TurtleBotTool(namespace="turtlebot2")  # B1 agent's TurtleBot
    turtlebot_tool_visitor = TurtleBotTool(namespace="turtlebot5")  # Visitor agent's TurtleBot

    # Attach tools to agents
    ci_agent.tools = [turtlebot_tool_ci]
    b1_agent.tools = [turtlebot_tool_b1]
    visitor_agent.tools = [turtlebot_tool_visitor]

    # === Define Tasks ===
    
    # CI agent navigates from the campus entrance to a building (e.g., B1)
    navigate_task = Task(
        description="Navigate from campus entrance to building. Use OpenAI for guidance.",
        expected_output="Successfully reach the building's coordinates.",
        agent=ci_agent
    )

    # B1 agent moves from the building entrance to the host coordinate inside the building
    move_to_host_task = Task(
        description="Move from building entrance to host location inside the building.",
        expected_output="Successfully reach the host location.",
        agent=b1_agent
    )

    # Visitor agent follows the CI agent to the building
    follow_ci_task = Task(
        description="Follow the CI agent to the building and host location. Use Serper to search for directions.",
        expected_output="Reach the host location following the CI agent.",
        agent=visitor_agent
    )

    # === Define the Crew (Multiple Agents and Tasks) ===
    
    # Group the agents and tasks into a crew
    crew = Crew(
        agents=[ci_agent, b1_agent, visitor_agent],
        tasks=[navigate_task, move_to_host_task, follow_ci_task],
        process=Process.sequential  # Execute tasks sequentially
    )

    # === Execute the Crew Workflow ===
    
    result = crew.kickoff(inputs={})  # Optionally pass inputs (like specific start/goal locations)
    print(result)

    # === Examples of Using API Keys ===
    
    # Example OpenAI prompt usage
    openai_result = turtlebot_tool_ci.query_openai("Provide navigation guidance for a robot.")
    print(f"OpenAI Result: {openai_result}")
    
    # Example Serper search query
    serper_result = turtlebot_tool_visitor.query_serper("directions to building B1")
    print(f"Serper Search Result: {serper_result}")

if __name__ == "__main__":
    main()
