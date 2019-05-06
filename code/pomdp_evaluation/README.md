# APPL_POMDPx_stimuli_automation
This is a bunch of scripts that allow to easily and automatically stimulate and collect the results of POMDPs following the APPL POMDPx file format.

Find more info about the **Approximate POMDP planning toolkit (APPL)** here:  http://bigbird.comp.nus.edu.sg/pmwiki/farm/appl/index.php?n=Main.HomePage

## Dependencies

* Script only works with pyhton2.7
* pandas needs to be installed for python 2.7 e.g. by executing `python2.7 -m pip install pandas`

## How to use it
1. Install APPL following their install guide (modified version of evaluator is used that prints actions taken to a .csv file) 
1. Set the correct paths to your POMDPx files and the place you want to store your results in the **evaluate_pomdps.py** file.
   The paths have to be set in two places : a) at the top of the **evaluate_pomdps.py**   b) everywhere you find a **TODO CHANGE PATH HERE** tag within the **evaluate_pomdps.py**
1. define your stimuli as a python script within the **stimuli** folder. 
1. specify the POMDPx models and the associated stimuli in the **POMDP_evaluation_config.csv**. 
1. Run the evaluate_pomdps.py file by `python evaluate_pomdps.py`


## Extracting stimuli from recorded bag files 

* Record the topic **/observation_agent/observation_update** to a rosbag. It contains the compiled observation in the POMDP readable format that is also sent to DESPOT for decision making. 
* Start the **bagextraction node** (roscore needs to run as well: 
```
rosrun hrc_ros bagfile_extractor.py
```
* The observations will be written to a **bag_extraction.csv** file.
* Save the file with another name - as the bag_extraction.csv file will be overwritten at the start of the script



