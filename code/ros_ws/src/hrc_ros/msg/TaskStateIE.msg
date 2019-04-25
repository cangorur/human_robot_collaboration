uint32 task_id
uint32 subtask_id # prev: subtask_counter

# next step is passed when human took a new action
uint32 step_count
string who_reports
time update_received_time

# below is common for the robot and the human
string human_model
# string human_expertise
# string human_mood
# string human_trust
time action_taken_time # time that human or robot took the action decision and started executing
string taken_action
string belief_state
string real_state
bool isEstimationCorrect
# this var is double TODO decide which to remove
uint32 warnings_count_subtask # number of warnings from human per subtask

# below is only for observations - received after observation update by observationManager
string  real_obs_received
uint8[] real_obs_received_array
string obs_with_noise # this is simulation specific
bool[] human_observables # this is for creating training set for type estimation
# this var is double TODO decide which one to remove
#ADDED for interaction experiment
uint32 mapped_observation_pomdp
uint32 mapped_observation_raw


# below are specific to robot status update
string robot_model
string immediate_reward
string total_disc_reward
float32[] robot_belief

# below is only informed when tray is updated
#ADDED SuccessStatusObserved statistics for interaction experiment - collected after tray update is received - see SuccessStatusObserved.msg
time tray_update_received_time
string subtask_status
string  who_succeeded_subtask
float64 subtask_duration

# below is only for manager reporting after each task
uint32  failed_subtasks
uint32  successful_subtasks
float64 percentage_successful_subtasks
string task_status # success, fail or ongoing
float64 task_duration
string  who_succeeded_task # if the task has been succeeded, who did it? In simu. if task is a success, AND human_achieves_subtasks > 5 then it is human, or else it is robot
uint32  warnings_count_task
uint32  successful_tasks_cnt
uint32  failed_tasks_cnt
float64 percentage_successful_tasks
