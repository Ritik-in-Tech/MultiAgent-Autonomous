# MultiAgent-Autonomous

This assignment focusses on the navigation of turtlebot in the gazebo wrold with the help of the ROS and crew AI.

## Installation

To run the ROS gazebo you have to setup the ros neotic and ro run ros you have Ubuntu20.04 version and you can use ROS2 for ubuntu22.04.

Please find the Link to setup ros-neotic. You can use ubuntu or run in the wsl for the setup of the ROS.
[Setup](https://roboticsclubiitj.github.io/Ros-Challenges/)

Here you have to go with the zero challenge, alpha challenge , and beta challenge.

After successful setup you can run the project.
Commands to run the project. There will be two termianl required to run the turtlebot in gazebo.

First Terminal Command

```bash
cd catkin_ws
```

```bash
source devel/setup.bash
```

```bash
catkin_make
```

```bash
roslaunch turtlebot3_gazebo autonomous.launch
```

Please make sure you have the autonomous.luanch file in the launch file which you can get from the repository.

On the second Terminal

```bash
cd catkin_ws
```

```bash
source devel/setup.bash
```

```bash
catkin_make
```

```bash
rosrun turtlebot3_gazebo move.py
```

Please make sure you have created one directory named scripts inside the turtlebot3_gazebo directory and inside that scripts folder create one file named move.py. The code for the move.py you will get from this repo. Please make sure you have the worlds and models all are setup. Hope you will get the right command to run this project.

To run the crew Ai python main.py you have to first declare the .env in crewai_project/turtlebot_navigation where you have mention two keys. For the security purposes I have put the .env on .gitignore.

.env content

```bash
OPENAI_API_KEY='Your_openai_key'
SERPER_API_KEY='Your_serper_api_key'
```

You can get seper API key from this url [Serper_Key](https://serper.dev/api-key) and the OpenAI API key from [Open_API](https://platform.openai.com/api-keys)

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
