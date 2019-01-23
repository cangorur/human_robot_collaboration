S = "null"
a = "null"
S1 = "null"
r = "null"
# (S, a, S1, r) the tuple you need for texplore at every step

# each row is reprented with the agent publisihing it, step_count and column_id: row["step_id"]["agent_id"]["column_id"]

n = 1  # step count. Ignore the 0th step always
for n <= max_step_number:
    if n == 1:
        S = from6Dto7D("True, False, False, False, False, True")  # for each n = 1, we take this constant observation, cause it is always the case initially (TaskHuman state)
        S1 = from6Dto7D(row[ n ]["OBSERVATION"]["human_observables"])
    elif(n == max_step_number)):
        S = from6Dto7D(row[ n-1 ]["OBSERVATION"]["human_observables"]) # take the (n-1)th step of observation agent
        S1 = from6Dto7D(row[ n-1 ]["OBSERVATION"]["human_observables"] and row[n-1]["SENSOR"])  # It seems that the last step always have only ROBOT and HUMAN and previous one always have SENSOR
    else:
        S = from6Dto7D(row[ n-1 ]["OBSERVATION"]["human_observables"]) # take the (n-1)th step of observation agent
        S1 = from6Dto7D(row[ n ]["OBSERVATION"]["human_observables"])

    a = row[ n ]["ROBOT"]["action_taken"]

    r = extractReward(row[ n ]["HUMAN"]["real_state"]) # you were right about this

    print("FOR STEP COUNT", n, "THE TUPLE IS", (S, a, S1, r) )


def from6Dto7D("human_observables"):  # mapping of human_observables to observation vector
    do the mapping here ...

def extractReward("human_observables"):  # deciding the reward amount
    if warning:
        assign reward
    if globalSuccess:
        ...
        ...
