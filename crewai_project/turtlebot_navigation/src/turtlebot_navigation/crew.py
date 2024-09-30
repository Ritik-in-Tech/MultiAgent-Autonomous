from crewai import Agent, Task, Crew, Process
from tools.turtlebot_tool import TurtleBotTool  # Import your custom TurtleBot tool

# === Define Agents ===
ci_agent = Agent(
    role="Campus Intelligence Agent",
    goal="Navigate across the campus and guide other agents",
    verbose=True
)

b1_agent = Agent(
    role="Building 1 Agent",
    goal="Move to host location inside the building",
    verbose=True
)

visitor_agent = Agent(
    role="Visitor Agent",
    goal="Follow CI agent to the host location",
    verbose=True
)

# === Instantiate the custom TurtleBot tools ===
turtlebot_tool_ci = TurtleBotTool(namespace="turtlebot1")  # For CI agent
turtlebot_tool_b1 = TurtleBotTool(namespace="turtlebot2")  # For B1 agent
turtlebot_tool_visitor = TurtleBotTool(namespace="turtlebot5")  # For Visitor agent

# === Add the TurtleBot tools to the agents ===
ci_agent.tools = [turtlebot_tool_ci]  # Assign the TurtleBot tool to CI agent
b1_agent.tools = [turtlebot_tool_b1]  # Assign the TurtleBot tool to B1 agent
visitor_agent.tools = [turtlebot_tool_visitor]  # Assign the TurtleBot tool to Visitor agent

# === Define Tasks ===
navigate_task = Task(
    description="Navigate from campus entrance to building",
    expected_output="Successfully reach the building's coordinates.",
    agent=ci_agent
)

move_to_host_task = Task(
    description="Move from building entrance to host location inside the building",
    expected_output="Successfully reach the host location.",
    agent=b1_agent
)

follow_ci_task = Task(
    description="Follow the CI agent to the building and host location.",
    expected_output="Reach the host location following the CI agent.",
    agent=visitor_agent
)

# === Define the Crew ===
crew = Crew(
    agents=[ci_agent, b1_agent, visitor_agent],
    tasks=[navigate_task, move_to_host_task, follow_ci_task],
    process=Process.sequential  # Execute tasks sequentially
)

# === Kickoff the Crew workflow ===
result = crew.kickoff(inputs={})
print(result)
