# How to run the result generation
.bag files are automatically generated here after each run of the system. It tracks the topic ´/task_manager/task_status´
The auto created files named with the format: ´tests_<date-and-time>.bag´. The initial name ´tests´ is defined manually under ´hrc.launch´ line 15. Change the name at the end of the argument path to indicate different experiments.

## Creating .csv files

In the ´task_status´ topic there are 5 agents reporting separately: *MANAGER, HUMAN, ROBOT, OBSERVATION, SENSORS*
Run first below to safely convert .bag file to .csv

```sh
python bag_to_csv.py <name_of_the_bag_file.bag>
```

This creates a folder with the same name of the .bag file. Now we should separate ROBOT, HUMAN, OBSERVATION and SENSOR information

### Saving Observations and Task Status
First of all we should separate task status information under a new .csv. This information is provided by *SENSORS* agent
´update_sensor_data.py´ script is created for this purpose. It works with 2 different modes that expects to be stated when run:
- save_sensors: Separates the sensor reports
- save_human_obs: Compiles a new file with human observables [Detected?, Looking around?, Graspped successfully?, Failed to grasp?, Warning?, Staying idle?]
- task_status: Extracts the task state information gathered from the sensors, Success or Fail. Adds the time a task took

```sh
python update_sensor_data.py <name_of_the_folder_created> save_sensors
python update_sensor_data.py <name_of_the_folder_created> save_human_obs
python update_sensor_data.py <name_of_the_folder_created> task_status
```

### Saving Human Results
Second of all we should separate human information under a new .csv. This information is provided by *HUMAN* agent.
´update_sensor_data.py´ script is created for this purpose. It works with 2 different modes that expects to be stated when run:
- save_human: Separates the human reports
- append_human: Appends the human states observed for distribution analysis
- state_distribution: Adds up the states human was in and creates total distribution for each task

```sh
python update_human_data.py <name_of_the_folder_created> save_human
python update_human_data.py <name_of_the_folder_created> append_human
python update_human_data.py <name_of_the_folder_created> state_distribution
```

### Saving Results for Robot
Finally we get the robot results. Those info provided by *ROBOT* agent.
´update_robot_data.py´ script is created for this purpose. It works with 3 different modes that expects to be stated when run:
- save_robot: Separates the robot reports
- robot_info: Gets only the important robot information from the raw data
- robot_results: Calculates total disc rewards for each task and estimation accuracy (*this is not providing correct values currently*)

```sh
python update_robot_data.py <name_of_the_folder_created> save_robot
python update_robot_data.py <name_of_the_folder_created> robot_info
python update_robot_data.py <name_of_the_folder_created> robot_results
```
