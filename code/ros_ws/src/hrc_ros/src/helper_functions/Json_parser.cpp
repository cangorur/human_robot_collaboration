#include <helper_functions/Json_parser.h>



namespace pt = boost::property_tree; 
using boost::property_tree::ptree;
using namespace std; 

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

/*vector<task_set> get_success_criteria_task(string str_task, string str_subtask,boost::property_tree::ptree config_pt){

    // first get subtask_chooser that decides wether multiple rules for different subtasks are present or if all rules are the same! 
    string subtask_path = string("task.") + str_task + string(".subtask"); 
    string object_path;
    task_set task_set_read;
    vector <task_set> task_set_vect; 
    vector <success_combo> success_combo_read;

    BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(subtask_path)){ // looping trough subtasks
            
            string subtask_chooser = child_object.first.data();
            if (subtask_chooser.compare("all") == 0){
                object_path = subtask_path + string(".") + string("all"); 
                cout << " object_path_all  : "  << object_path << endl;
                break;
            } else {
                object_path = subtask_path + string(".") + str_task;
                 cout << " object_path_subtasks  : "  << object_path << endl; 
            }
        }


        BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(object_path)){ // looping trough subtasks
            
            string str_object = child_object.first.data(); 
            string str_tray   = config_pt.get<string>(( object_path + string(".") + str_object + string(".tray") ) );
            //cout << " task: " << str_task << "  subtask: " << str_subtask << " object : " << str_object << " tray : " << str_tray << endl; 
            task_set_read.task = number_str_to_int(str_task);
            task_set_read.subtask = number_str_to_int(str_task);
            task_set_read.success_combo.object = object_str_to_int(str_object); 
            task_set_read.success_combo.tray   = std::stoi(str_tray);
            task_set_vect.push_back(task_set_read); 
          
        }

        return task_set_vect; 
}
*/


// ####################  get success criteria worked with ################################## 

/*vector<task_set> get_success_criteria_object(string str_task, string str_subtask,string str_object,boost::property_tree::ptree config_pt){

    // first get subtask_chooser that decides wether multiple rules for different subtasks are present or if all rules are the same! 
    string subtask_path = string("task.") + str_task + string(".subtask"); 
    string object_path;
    string subtask_quantity_string;
    task_set task_set_read;
    vector <task_set> task_set_vect; 
    success_combo current_success_combo; 

    task_set_read.subtask_quantity = stoi( config_pt.get<string>(( string("task.") + str_task + string(".subtask_quantity") ) )  );

    BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(subtask_path)){ // looping trough subtasks
            
            string subtask_chooser = child_object.first.data();



            if (subtask_chooser.compare("all") == 0){
                object_path = subtask_path + string(".") + string("all") + string(".") + str_object; 
                //cout << " object_path_all  : "  << object_path << endl;
                break;
            } else if (subtask_chooser.compare("subtask_count") != 0 ) {  // if not subtask_count 
                object_path = subtask_path + string(".") + str_subtask + string(".") + str_object;
                cout << " object_path_subtasks  : "  << object_path << endl; 
                break;
            }
        }


        BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object , config_pt.get_child(object_path)){ // looping trough subtasks
             
            string str_tray   = config_pt.get<string>(( object_path + string(".tray") ) );
            //cout << " task: " << str_task << "  subtask: " << str_subtask << " object : " << str_object << " tray : " << str_tray << endl;
            current_success_combo.object = object_str_to_int(str_object);
            current_success_combo.tray   = stoi(str_tray);
            task_set_read.task = stoi(str_task);
            task_set_read.subtask = stoi(str_subtask);
            task_set_read.success_combo_vect.push_back(current_success_combo);
            task_set_vect.push_back(task_set_read); 
          
        }

        return task_set_vect; 
}
*/




task_set read_task_set(std::string str_task,boost::property_tree::ptree config_pt){

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
                cout << " object_path_subtasks  : "  << object_path << endl;
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