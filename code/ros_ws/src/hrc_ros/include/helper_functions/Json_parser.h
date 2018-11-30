#include <fstream>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <string>


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

task_set read_task_set(std::string str_task,boost::property_tree::ptree config_pt);
std::vector<task_set> get_success_criteria(std::string str_task, std::string str_subtask, boost::property_tree::ptree config_pt);
std::vector<task_set> get_success_criteria_object(std::string str_task, std::string str_subtask,std::string str_object,boost::property_tree::ptree config_pt);
int object_str_to_int(std::string string_in);
void print_task_set(task_set task_rules);