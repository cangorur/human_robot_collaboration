uint32 task_id

# next step is passed when human took a new action
uint32 step_count
string who_reports
time update_received_time

# below is common for the robot and the human
string human_expertise
string human_mood
string human_trust
time action_taken_time # time that human or robot took the action decision and started executing
string taken_action
string belief_state
string real_state
bool isEstimationCorrect

# below is only for observations
string real_obs_received
string obs_with_noise
string task_status # success, fail or ongoing
string who_succeeded # if the task has been succeeded who did then

# below are specific to robot status update
string immediate_reward
string total_disc_reward
