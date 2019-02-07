/* Copyright 2019 Elia Jona Kargruber 
 *
 * This project is licensed under the terms of the MIT license.
 * 
*/

#include <helper_functions/Json_parser.h>

using namespace std; 

namespace pt = boost::property_tree; 
using boost::property_tree::ptree;

// Reading array with helper function
template <typename T>
std::vector<T> as_vector(pt::ptree const& ptree, pt::ptree::key_type const& key)
{
    std::vector<T> r;
    BOOST_FOREACH (pt::ptree::value_type const& item, ptree.get_child(key))
        r.push_back(item.second.get_value<T>());
    return r;
}


// helper functions
int object_str_to_int(std::string string_in){
    int return_int = 999; 
    if (string_in.compare("object_0") == 0){
        return_int = 0; 
    } else if (string_in.compare("object_1") == 0){
        return_int = 1; 
    } else if (string_in.compare("object_2")==0){
        return_int = 2; 
    } else if (string_in.compare("object_3")==0){
        return_int = 3; 
    } else if (string_in.compare("object_4")==0){
        return_int = 4; 
    } else if (string_in.compare("object_5")==0){
        return_int = 5; 
    } else if (string_in.compare("object_6")==0){
        return_int = 6; 
    } else if (string_in.compare("object_7")==0){
        return_int = 7; 
    } else if (string_in.compare("object_8")==0){
        return_int = 8; 
    } else if (string_in.compare("object_9")==0){
        return_int = 9; 
    } else if (string_in.compare("object_10")==0){
        return_int = 10; 
    } else return_int = 999; // ERROR! 

    return return_int; 
}


// helper functions
string object_int_to_str(int obj_int_in){
    string return_str = "EMPTY"; 
    if (obj_int_in == 0){
        return_str = "object_0"; 
    } else if (obj_int_in == 1){
        return_str = "object_1"; 
    } else if (obj_int_in == 2){
        return_str = "object_2"; 
    } else if (obj_int_in == 3){
        return_str = "object_3";
    } else if (obj_int_in == 4){
        return_str = "object_4"; 
    } else if (obj_int_in == 5){
        return_str = "object_5"; 
    } else if (obj_int_in == 6){
        return_str = "object_6"; 
    } else if (obj_int_in == 7){
        return_str = "object_7"; 
    } else if (obj_int_in == 8){
        return_str = "object_8"; 
    } else if (obj_int_in == 9){
        return_str = "object_9";                                              
    } else return_str = "EMPTY"; // ERROR! 

    return return_str; 
}







void print_task_set(task_set task_rules){

    for ( int subt_i =0; subt_i < (task_rules.subtask_set_vector.size()) ; subt_i ++){

        task_set print_task_set = task_rules;
        cout << "####### Print current task set ### " << endl << "  task_set : " << subt_i << "   |  task : " << print_task_set.task << "   subtask: " << print_task_set.subtask_set_vector.at(subt_i).subtask << "   subtask_quantity  : " << print_task_set.subtask_quantity  << endl;
        for ( int combo_j = 0; combo_j < print_task_set.subtask_set_vector.at(subt_i).success_combo_vect.size(); combo_j++){
            cout << "  success_criteria: " << print_task_set.subtask_set_vector.at(subt_i).success_combo_vect.at(combo_j).object << "  tray:  " << print_task_set.subtask_set_vector.at(subt_i).success_combo_vect.at(combo_j).tray << endl;
        } 
        cout << endl << endl; 
    }
}

/*********************************************************************** 
 * 
 * boost property tree is searched for a specific task_set and the task_set is returned 
 * 
 */
task_set read_task_set(string str_task,boost::property_tree::ptree config_pt){

    // first get subtask_chooser that decides wether multiple rules for different subtasks are present or if all rules are the same! 
    string subtask_path = string("task.") + str_task + string(".subtask"); 
    string object_path;
    string subtask_quantity_string;
    task_set task_set_read;
    
    vector <task_set> task_set_vect; 
    

    task_set_read.subtask_quantity = stoi( config_pt.get<string>(( string("task.") + str_task + string(".subtask_quantity") ) )  );
    task_set_read.task = stoi(str_task);

    BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(subtask_path)){ // looping trough subtasks
            
            string subtask_chooser = child_object.first.data();
            subtask_set subtask_set_read;

            if (subtask_chooser.compare("all") == 0){
                object_path = subtask_path + string(".") + string("all") + string("."); 
                //cout << " object_path_all  : "  << object_path << endl;
                task_set_read.all_set = true; 

                // ++++++ looping trough subtask
                BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(object_path)){ // looping trough subtasks
                    success_combo current_success_combo; 

                    string object_at_hand_str = child_object.first.data(); 
                    string str_tray   = config_pt.get<string>(( object_path + object_at_hand_str + string(".tray") ) );
                    //cout << " task: " << str_task << "  subtask: " << str_subtask << " object : " << str_object << " tray : " << str_tray << endl;
                    

                    current_success_combo.object = object_str_to_int(object_at_hand_str);
                    current_success_combo.tray   = stoi(str_tray);
                    
                    // ** combile subtask_set | task | success_combo_vect | task 
                    subtask_set_read.task = stoi(str_task);
                    subtask_set_read.subtask = 99 ;
                    subtask_set_read.success_combo_vect.push_back(current_success_combo);
                     
                }
                task_set_read.subtask_set_vector.push_back(subtask_set_read);
                
            } else if (subtask_chooser.compare("subtask_count") != 0 ) {  // if not subtask_count 
                object_path = subtask_path + string(".") + subtask_chooser + string(".");
                //cout << " object_path_subtasks  : "  << object_path << endl;
                task_set_read.all_set = false;

                BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(object_path)){ // looping trough subtasks
                    
                    success_combo current_success_combo;

                    string object_at_hand_str = child_object.first.data(); 
                    string str_tray   = config_pt.get<string>(( object_path + object_at_hand_str + string(".tray") ) );
                    //cout << " task: " << str_task << "  subtask: " << str_subtask << " object : " << str_object << " tray : " << str_tray << endl;
                    

                    current_success_combo.object = object_str_to_int(object_at_hand_str);
                    current_success_combo.tray   = stoi(str_tray);
                    
                    // ** combile subtask_set | task | success_combo_vect | task 
                    subtask_set_read.task = stoi(str_task);
                    subtask_set_read.subtask = stoi(subtask_chooser);
                    subtask_set_read.success_combo_vect.push_back(current_success_combo);

                     
                }

                task_set_read.subtask_set_vector.push_back(subtask_set_read);
  
            }
        }

        return task_set_read; 
}

int get_subtask_quantity(string str_task,boost::property_tree::ptree config_pt){

    string path = "task." + str_task + ".subtask_quantity"; 
    int subtask_quantity = 0; 

    subtask_quantity = stoi(config_pt.get<string>(path));
    return subtask_quantity; 

}

global_task_config read_global_task_config(boost::property_tree::ptree config_pt){

    global_task_config task_config_read;

    task_config_read.subtask_max = stoi(config_pt.get<string>("config.subtask_max"));
    task_config_read.task_max    = stoi(config_pt.get<string>("config.task_max"));
    task_config_read.global_success_assert = stoi(config_pt.get<string>("config.global_success"));
    task_config_read.global_fail_assert = stoi(config_pt.get<string>("config.global_fail"));
    task_config_read.sameaction_timeout = stod(config_pt.get<string>("config.same_action_timeout"));
    task_config_read.decision_timer_periode = stod(config_pt.get<string>("config.decision_timer_periode"));

    return task_config_read; 
}

/*********************************************************************** 
 * 
 * boost property tree is searched for a specific success criteria combination
 * consitint of the correct object tray combination.
 * Inputs: task, subtask, object, property_tree
 * Outputs: success_combo
 * 
 */
success_combo get_success_criteria(string str_task,string subtask_str, string object_str, boost::property_tree::ptree config_pt){

    // first get subtask_chooser that decides wether multiple rules for different subtasks are present or if all rules are the same! 
    string subtask_path = string("task.") + str_task + string(".subtask"); 
    string object_path;
    string subtask_quantity_string;
    task_set task_set_read;
    success_combo success_combo_read; 
    
    vector <task_set> task_set_vect; 
    

    //task_set_read.subtask_quantity = stoi( config_pt.get<string>(( string("task.") + str_task + string(".subtask_quantity") ) )  );
    //task_set_read.task = stoi(str_task);

    BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(subtask_path)){ // looping trough subtasks
            
            string subtask_chooser = child_object.first.data();
            subtask_set subtask_set_read;

            if (subtask_chooser.compare("all") == 0){
                object_path = subtask_path + string(".") + string("all") + string(".") + object_str; 
                //cout << " object_path_all  : "  << object_path << endl;


                int success_tray = stoi ( config_pt.get<string>(( object_path  + string(".tray") ))  );
                int success_object = object_str_to_int(object_str);
                
                
                success_combo_read.tray = success_tray;
                success_combo_read.object = success_object;  
                break; 
                
            } else if (subtask_chooser.compare("subtask_count") != 0 ) {  // if not subtask_count 
                object_path = subtask_path + string(".") + subtask_str + string(".") + object_str;
                //cout << " object_path_subtasks  : "  << object_path << endl;


                int success_tray = stoi ( config_pt.get<string>(( object_path  + string(".tray") ))  );
                int success_object = object_str_to_int(object_str);
                
                
                success_combo_read.tray = success_tray;
                success_combo_read.object = success_object; 
                break; 
              
  
            }
        }

        return success_combo_read; 
}
