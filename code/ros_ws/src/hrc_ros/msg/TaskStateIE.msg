uint32 task_id

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
uint32 warnings_count # number of warnings from human


# below is only for observations - received after observation update by observationManager
string  real_obs_received
uint8[] real_obs_received_array
bool[] human_observables # this is for creating training set for type estimation
string task_status # success, fail or ongoing
# this var is double TODO decide which one to remove
#string who_succeeded # if the task has been succeeded who did then
#ADDED for interaction experiment
uint32 mapped_observation_pomdp
uint32 mapped_observation_raw


# below are specific to robot status update
string robot_model
string immediate_reward
string total_disc_reward
float32[] robot_belief

#ADDED SuccessStatusObserved statistics for interaction experiment - collected after tray update is received - see SuccessStatusObserved.msg
time tray_update_received_time
string subtask_success_status
string task_success_status
uint32 task_counter
uint32 subtask_counter
uint32  failed_subtasks
uint32  successful_subtasks
float64 subtask_time_seconds
float64 task_combined_subtask_time_seconds
float64 percentage_successful_subtasks
string  who_succeeded
uint32  task_warnings_received
uint32  successful_tasks_cnt
uint32  failed_tasks_cnt
float64 percentage_successful_tasks
