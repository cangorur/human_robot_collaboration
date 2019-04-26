/* Copyright 2019 Elia Jona Kargruber 
 *
 * This project is licensed under the terms of the MIT license.
 * 
*/

#include <fstream>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <string>

struct global_task_config {
    //int subtask_max;  -- not used anymore - define on task level 
    int task_max; 
    int global_success_assert; 
    int global_fail_assert; 
    double sameaction_timeout; 
    double decision_timer_periode; 
};

struct success_combo {
    int object, tray;
};


struct subtask_set {
    int task; 
    int subtask; 
    
    std::vector<success_combo> success_combo_vect;
};

struct task_set {
    int task;
    bool all_set; 
    int subtask_quantity;
    std::vector <subtask_set> subtask_set_vector; 
};

// read complete taks set 
task_set read_task_set(std::string str_task,boost::property_tree::ptree config_pt);
//std::vector<task_set> get_success_criteria(std::string str_task, std::string str_subtask, boost::property_tree::ptree config_pt);
//std::vector<task_set> get_success_criteria_object(std::string str_task, std::string str_subtask,std::string str_object,boost::property_tree::ptree config_pt);

global_task_config read_global_task_config(boost::property_tree::ptree config_pt);

// getting a single success_criteria (right tray) according to the current task rules 
success_combo get_success_criteria(std::string str_task,std::string subtask_str,std::string object_str, boost::property_tree::ptree config_pt);
int get_subtask_quantity(std::string str_task,boost::property_tree::ptree config_pt);
std::string get_subtask_type(std::string str_task,boost::property_tree::ptree config_pt);

// Helper functions for string to int conversion and vice versa 
int object_str_to_int(std::string string_in);
std::string object_int_to_str(int obj_int_in);
void print_task_set(task_set task_rules);
