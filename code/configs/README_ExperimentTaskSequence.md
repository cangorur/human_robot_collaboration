## Sequence of the tasks 

|Task# 	|Brief 				            |Task type   	                              |Description / purpose   				|   	
|---	|---				            |---						                  |---									|
| 1  	|trial round   			        | 1     assign by colors - all rules the same |Clarify misunderstandings in task and counteract training effect    		|   	
| 2  	|Task type 1 no assistance   	| 1     assign by colors - all rules the same |Test how participants score without robot assistance - Easy type 1 task   		|   	
| 3  	|Task type 1 assistance		    | 1  	assign by colors - all rules the same |Inspect long term effects with task type 1 with assistance   		|  	
| 4	    |Task 2 again with assistance	| 1		assign by colors - all rules the same |Inspect if assistance is needed and prefered in such tasks & Inspect longterm effects 		|
| 5	    |Task type 2 with assistance    | 2		assign by order - repeat sequence     |Inspect how participants score in task type 2 with assistance		|
| 6	    |Task type 2 long term          | 2		assign by order - repeat sequence     |Inspect long term effects with task type 1 with assistance		|
| 7	    |Task type 3 with assistance    | 3		assign by color and order - 1 rule for each subtask     |Inspect how participants score in task type 3 with assistance		|
| 8	    |Task type 3 long term          | 3 	assign by color and order - 1 rule for each subtask     |Inspect long term effects with task type 3 with assistance		|

### How to specify task rules
Task rules can be specified in the IE_task_config.json file. The following can be specified. 

* In the **config** part on the top you can define the following 
    * **task_max:** How many tasks should be executed at most
    * **global_success:** How many subtasks need to be correct in order to count the task as a success (e.g. 7 subtasks need to be correct)
    * **global_fail:**    How many subtasks need to be false in order to count the task as a failure - should be subtasks - global_success
    * **same_action_timeout:** Time in seconds after which a new decision will be triggered, even if the human action observation has not changed (in the setup 3.0 works well)
    * **decision_timer_periode:** Time in seconds after which a new decision will be triggered even if no observation or tray update has been received.
    * **red_subtasks_order:** Array of subtask numbers where a red object occurs. This specifies the sequence of objects that should be placed on the conveyor and is used for task type 3 rueles. (e.g. ["1", "4", "7"] means that the 1st, 4th and 7th object on the conveyor should be a red one)
    * **green_subtasks_order:** Array of subtask numbers where a green object occurs. This specifies the sequence of objects that should be placed on the conveyor and is used for task type 3 rueles.
    * **blue_subtasks_order:** Array of subtask numbers where a green object occurs. This specifies the sequence of objects that should be placed on the conveyor and is used for task type 3 rueles.

* There are 3 different task types: \
For all of them specify the number of subtasks with the **subtask_quantity** field and set the task type in the **subtask_type field** to any of the 3 following values: 

    * **1_all_rules_same** The objects are assigned by color. There are 3 rules that are valid for a whole task.  Please use subtask specifier "all". You only need to specify 3 rules, one for each color. 
    * **2_same_tray_order** The objects are assigned by order of occurence. You can specify the order of placement with the subtask rule 1-3, afterwards the placement order will be repeated, so make sure you repeate the same rule for subtasks 4-6 and 7-9 and 10. As the object colour is not taken into consideration, make sure that the tray rule for a specific subtask is the same for all 3 objects. 
    * **3_mixed_fixed_sequence** The object are assigned by color and by the order of occurence. You have to consider the object sequence specified with the xyz_subtask_order array. All 10 rules will be displayed. 

### Sequence of the objects placed onto the conveyor 
The sequence of objects can be specified in the IE_task_config.json file.
The sequence is important for task type 3.  Currently the following sequence is used:

R G B R B G R G B R 


