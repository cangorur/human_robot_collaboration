#include <despot/evaluator.h>

using namespace std;

namespace despot {

/* =============================================================================
 * EvalLog class
 * =============================================================================*/

typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;	// Qichao

time_t EvalLog::start_time = 0;
double EvalLog::curr_inst_start_time = 0;
double EvalLog::curr_inst_target_time = 0;
double EvalLog::curr_inst_budget = 0;
double EvalLog::curr_inst_remaining_budget = 0;
int EvalLog::curr_inst_steps = 0;
int EvalLog::curr_inst_remaining_steps = 0;
double EvalLog::allocated_time = 1.0;
double EvalLog::plan_time_ratio = 1.0;

EvalLog::EvalLog(string log_file) :
	log_file_(log_file) {
	ifstream fin(log_file_.c_str(), ifstream::in);
	if (!fin.good() || fin.peek() == ifstream::traits_type::eof()) {
		time(&start_time);
	} else {
		fin >> start_time;

		int num_instances;
		fin >> num_instances;
		for (int i = 0; i < num_instances; i++) {
			string name;
			int num_runs;
			fin >> name >> num_runs;
			runned_instances.push_back(name);
			num_of_completed_runs.push_back(num_runs);
		}
	}
	fin.close();
}

void EvalLog::Save() {
	ofstream fout(log_file_.c_str(), ofstream::out);
	fout << start_time << endl;
	fout << runned_instances.size() << endl;
	for (int i = 0; i < runned_instances.size(); i++)
		fout << runned_instances[i] << " " << num_of_completed_runs[i] << endl;
	fout.close();
}

void EvalLog::IncNumOfCompletedRuns(string problem) {
	bool seen = false;
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == problem) {
			num_of_completed_runs[i]++;
			seen = true;
		}
	}

	if (!seen) {
		runned_instances.push_back(problem);
		num_of_completed_runs.push_back(1);
	}
}

int EvalLog::GetNumCompletedRuns() const {
	int num = 0;
	for (int i = 0; i < num_of_completed_runs.size(); i++)
		num += num_of_completed_runs[i];
	return num;
}

int EvalLog::GetNumRemainingRuns() const {
	return 80 * 30 - GetNumCompletedRuns();
}

int EvalLog::GetNumCompletedRuns(string instance) const {
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == instance)
			return num_of_completed_runs[i];
	}
	return 0;
}

int EvalLog::GetNumRemainingRuns(string instance) const {
	return 30 - GetNumCompletedRuns(instance);
}

double EvalLog::GetUsedTimeInSeconds() const {
	time_t curr;
	time(&curr);
	return (double) (curr - start_time);
}

double EvalLog::GetRemainingTimeInSeconds() const {
	return 24 * 3600 - GetUsedTimeInSeconds();
}

// Pre-condition: curr_inst_start_time is initialized
void EvalLog::SetInitialBudget(string instance) {
	curr_inst_budget = 0;
	if (GetNumRemainingRuns() != 0 && GetNumRemainingRuns(instance) != 0) {
		cout << "Num of remaining runs: curr / total = "
			<< GetNumRemainingRuns(instance) << " / " << GetNumRemainingRuns()
			<< endl;
		curr_inst_budget = (24 * 3600 - (curr_inst_start_time - start_time))
			/ GetNumRemainingRuns() * GetNumRemainingRuns(instance);
		if (curr_inst_budget < 0)
			curr_inst_budget = 0;
		if (curr_inst_budget > 18 * 60)
			curr_inst_budget = 18 * 60;
	}
}

double EvalLog::GetRemainingBudget(string instance) const {
	return curr_inst_budget
		- (get_time_second() - EvalLog::curr_inst_start_time);
}

/* =============================================================================
 * Evaluator class
 * =============================================================================*/

Evaluator::Evaluator(DSPOMDP* model, string belief_type, Solver* solver,
	clock_t start_clockt, ostream* out) :
	model_(model),
	belief_type_(belief_type),
	solver_(solver),
	start_clockt_(start_clockt),
	target_finish_time_(-1),
	out_(out) {
}

Evaluator::~Evaluator() {
}


//==========================================================
void Evaluator::webSocketClient(int action, string belief_state, double reward, double discounted_reward) {
	typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;

	// send action to human actuator
  WsClient client("localhost:8080");
  
  client.on_open=[&]() {
	  
	  std::size_t index = belief_state.find(":") + 1;
	  string str = belief_state.substr(index);
	  string belief_state_str = str.substr(0, str.size()-1);
	  
	  std::ostringstream strs;
	  strs << reward;
	  std::string reward_str = strs.str();
	  
	  std::ostringstream strss;
	  strss << discounted_reward;
	  std::string disc_reward_str = strss.str();
	  
      string message = std::to_string(action) + "," + belief_state_str + "," + reward_str + "," + disc_reward_str;

      //*out_ << "Client: Opened connection" << endl;
      *out_ << ">>> Robot POMDP Client: Sending message: \"" << message << "\"" << endl;

      auto send_stream=make_shared<WsClient::SendStream>();
      *send_stream << message;
      client.send(send_stream);
  }; 

  client.on_message=[&client](shared_ptr<WsClient::Message> message) {
      //cout << "Client: Sending close connection" << endl;
      client.send_close(1000);
  };

  client.start();
}
//==========================================================

bool Evaluator::RunInitial() {
	int action = solver_->Search().action; // initial action selection
	Evaluator::webSocketClient(action, state_->text(), 0.0, 0.0);
	previous_action = action;
	*out_ << "Robot Took Action: " << action << "in init state: " << state_->text() << endl;
	return true;
}

bool Evaluator::RunStep(int step, int round, int real_state, int manual_obs) {
	if (target_finish_time_ != -1 && get_time_second() > target_finish_time_) {
		if (!Globals::config.silence && out_)
			*out_ << "Exit. (Total time "
				<< (get_time_second() - EvalLog::curr_inst_start_time)
				<< "s exceeded time limit of "
				<< (target_finish_time_ - EvalLog::curr_inst_start_time) << "s)"
				<< endl
				<< "Total time: Real / CPU = "
				<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
				<< (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
				<< endl;
		exit(1);
	}
	
	OBS_TYPE obs = manual_obs;
	double start_t;
	double end_t;
	double step_start_t;
	double step_end_t;
	int sim_type = 2;
	double reward;
	
	*out_ << "-----------------------------------Round " << round
							<< " Step " << step + 1 << "-----------------------------------"
							<< endl;
	*out_ << "- The real state received: " << real_state << endl;
	*out_ << "- Observation received from websocket: " << manual_obs << endl;
	*out_ << "=== RESULTS ===" << endl;
	
	// ################# REWARD CALCULATION STARTS ####################//
	if (real_state != -1){ // IF REAL STATE INFORMATION RECEIVED THEN CALCULATE THE REWARD
		// ########## SIMULATOR STARTS ########### //
		// to get the reward, first simulator runs ! The new state only gives the reward of our previous act!
		start_t = get_time_second();
	
		//== Executing the action here ==//
		bool terminal = ExecuteAction(sim_type, real_state, previous_action, reward, obs);
		// terminal = false; // TODO: we disabled the termination of the states for continuous testing
		// TODO: Since the terminal states are hard to define in a rewarding system automatically, here we manually check
		// Terminal states are: GlobalSuccess and GlobalFail
		terminal = (real_state == 8 || real_state == 9) ? true : false;
		ReportStepReward();
	
		end_t = get_time_second();
		logi << "[RunStep] Time spent in ExecuteAction(): " << (end_t - start_t)
			<< endl;
		//== Action was executed ==//
		
		// ########## SENDING THE RESULTS TO THE SOCKET ########### //
		// Sending rewards gathered from the new state
		Evaluator::webSocketClient(-1, state_->text(), reward_, total_discounted_reward_);
		
		if (terminal) {
			step_end_t = get_time_second();
			logi << "[RunStep] Time for step: actual / allocated = "
				<< (step_end_t - step_start_t) << " / " << EvalLog::allocated_time
				<< endl;
			if (!Globals::config.silence && out_)
				*out_ << endl;
			step_++;
			return true;
		}
		// ########## SIMULATOR ENDS ########### //
	}
	// ################# REWARD CALCULATION ENDS ####################//
	
	// ################# BELIEF UPDATE AND ACTION SELECTION STARTS ####################//
	if (manual_obs != -1){
		// ########## AGENT BELIEF UPDATE STARTS########## //
		if (!Globals::config.silence && out_) {
			*out_ << "- Observation = ";
			model_->PrintObs(*state_, obs, *out_);
		}
	
		if (state_ != NULL) {
			if (!Globals::config.silence && out_)
				*out_ << "- ObsProb = " << model_->ObsProb(obs, *state_, previous_action)
					<< endl;
		}
		
		start_t = get_time_second();
		solver_->Update(previous_action, obs);
		
		end_t = get_time_second();
		logi << "[RunStep] Time spent in Update(): " << (end_t - start_t) << endl;
		
		//TODO: added for the purpose of printing the particle distribution
		/*vector<pair<string, double>> particle_distr = solver_->belief_distr;;
		ostringstream oss;
		cout << "- Agent's belief:";
		for (int i = 0; i < particle_distr.size(); i++) {
			pair<string, double> pair = particle_distr[i];
			oss += pair.first + "=" + pair.second + ":";
			cout << oss.str();
		}*/
		// ########## AGENT BELIEF UPDATE ENDS ########## //
	
		
		// ########## AGENT'S ACTION SELECTION STARTS ########## //
		step_start_t = get_time_second();
		start_t = get_time_second();
		int action = solver_->Search().action;
		previous_action = action; // THIS ACTION WILL BE USED IN THE NEXT STEP TO CALCULATE THE BELIEF WHEN NEW OBS ARRIVES
		end_t = get_time_second();
	
		logi << "[RunStep] Time spent in " << typeid(*solver_).name()
			<< "::Search(): " << (end_t - start_t) << endl;
	
		if (!Globals::config.silence && out_) {
			*out_ << "- Agent Acts = ";
			model_->PrintAction(action, *out_);
		}
		// ########## AGENT'S ACTION SELECTION ENDS ########## //
	
		
		// ########## SENDING THE RESULTS TO THE SOCKET ########### //

		vector<pair<string, double>> belief_distr = solver_->GetBeliefDistribution();	

		int index_first = 0;
		for (int i = 0; i < belief_distr.size(); i++) {
			if (belief_distr[i].second >= belief_distr[index_first].second)
				index_first = i;
		}
		*out_ << "- Belief state with probability: " << belief_distr[index_first].first << " = " << belief_distr[index_first].second << endl;
		*out_ << "- FINAL: Action taken: " << action << " in the belief state: " << belief_distr[index_first].first << endl;
		Evaluator::webSocketClient(action, belief_distr[index_first].first, 0.0, 0.0);
		// ######################################################## //
	}
	// ################# BELIEF UPDATE AND ACTION SELECTION ENDS ####################//
	
	*out_<<endl;
	if (manual_obs != -1) // step counts every time a new observation is received
		step_++;
	
	return false;
}

double Evaluator::AverageUndiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
	}
	return undiscounted_round_rewards_.size() > 0 ? (sum / undiscounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrUndiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = undiscounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}


double Evaluator::AverageDiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
	}
	return discounted_round_rewards_.size() > 0 ? (sum / discounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrDiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = discounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

void Evaluator::ReportStepReward() {
	// NOTE: The rewards are incorrectly computed at last step in IPPC
	if (!Globals::config.silence && out_)
		*out_ << "- Reward = " << reward_ << endl
			<< "- Accumulated rewards:" << endl
			<< "  discounted / undiscounted = " << total_discounted_reward_
			<< " / " << total_undiscounted_reward_ << endl;
}

/* =============================================================================
 * IPPCEvaluator class
 * =============================================================================*/

IPPCEvaluator::IPPCEvaluator(DSPOMDP* model, string belief_type, Solver* solver,
	clock_t start_clockt, string hostname, string port, string log,
	ostream* out) :
	Evaluator(model, belief_type, solver, start_clockt, out),
	pomdpx_(static_cast<POMDPX*>(model)),
	log_(log),
	hostname_(hostname),
	port_(port) {
}

IPPCEvaluator::~IPPCEvaluator() {
}

int IPPCEvaluator::Handshake(string instance) {
	int num_remaining_runs = log_.GetNumRemainingRuns(instance);
	if (num_remaining_runs == 0) {
		return 0;
	}

	double start_t = get_time_second();
	instance_ = instance;

	client_ = new Client();
	client_->setHostName(hostname_);
	client_->setPort(port_);

	client_->initializeSocket();
	client_->connectToServer();

	client_->sendMessage(client_->createSessionRequestMes(instance));

	string sessionInitMes = client_->recvMessage();

	if (!Globals::config.silence && out_) {
		*out_ << sessionInitMes << endl;
	}

	client_->processSessionInitMes(sessionInitMes);
	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for handsake " << (end_t - start_t) << endl;
	}

	log_.SetInitialBudget(instance);
	EvalLog::curr_inst_steps = num_remaining_runs * Globals::config.sim_len;
	EvalLog::curr_inst_remaining_steps = num_remaining_runs
		* Globals::config.sim_len;
	EvalLog::curr_inst_target_time = EvalLog::curr_inst_budget
		/ EvalLog::curr_inst_steps;
	UpdateTimeInfo(instance);
	EvalLog::plan_time_ratio = 1.0;
	Globals::config.time_per_move = EvalLog::plan_time_ratio
		* EvalLog::allocated_time;

	return num_remaining_runs;
}

void IPPCEvaluator::InitRound() {
	step_ = 0;
	state_ = NULL;

	double start_t, end_t;

	// Initial belief
	start_t = get_time_second();
	delete solver_->belief();
	end_t = get_time_second();
	logi << "[IPPCEvaluator::InitRound] Deleted initial belief in "
		<< (end_t - start_t) << "s" << endl;

	start_t = get_time_second();
	Belief* belief = model_->InitialBelief(NULL, belief_type_);
	end_t = get_time_second();
	logi << "[IPPCEvaluator::InitRound] Initialized initial belief: "
		<< typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;

	solver_->belief(belief);

	// Initiate a round with server
	start_t = get_time_second();
	client_->sendMessage(client_->createRoundRequestMes());
	string roundMes = client_->recvMessageTwice();
	end_t = get_time_second();
	logi << "[IPPCEvaluator::InitRound] Time for startround msg "
		<< (end_t - start_t) << "s" << endl;

	total_discounted_reward_ = 0;
	total_undiscounted_reward_ = 0;
}

double IPPCEvaluator::EndRound() {
	double start_t = get_time_second();

	string roundEndMes = client_->recvMessage();
	double round_reward = client_->processRoundEndMes(roundEndMes);

	if (!Globals::config.silence && out_) {
		*out_ << "Total undiscounted reward = " << round_reward << endl;
	}

	log_.IncNumOfCompletedRuns(instance_);
	log_.Save();

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for endround msg (save log) " << (end_t - start_t)
			<< endl;
	}

	discounted_round_rewards_.push_back(total_discounted_reward_);
	undiscounted_round_rewards_.push_back(round_reward);

	return round_reward;
}

bool IPPCEvaluator::ExecuteAction(int action, double& reward, OBS_TYPE& obs) {
	double start_t = get_time_second();

	client_->sendMessage(
		client_->createActionMes(pomdpx_->GetActionName(),
			pomdpx_->GetEnumedAction(action)));

	if (step_ == Globals::config.sim_len - 1) {
		return true;
	}

	string turnMes = client_->recvMessage();

	//get step reward from turn message: added by wkg
	reward = client_->getStepReward(turnMes);
	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;

	map<string, string> observs = client_->processTurnMes(turnMes);
	obs = pomdpx_->GetPOMDPXObservation(observs);

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for executing action " << (end_t - start_t) << endl;
	}

	return false;
}

double IPPCEvaluator::End() {
	double start_t = get_time_second();

	string sessionEndMes = client_->recvMessage();
	double total_reward = client_->processSessionEndMes(sessionEndMes);
	client_->closeConnection();
	delete client_;

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for endsession " << (end_t - start_t) << endl
			<< "Total reward for all runs = " << total_reward << endl
			<< "Total time: Real / CPU = "
			<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
			<< (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
			<< endl;
	}

	return total_reward;
}

void IPPCEvaluator::UpdateTimeInfo(string instance) {
	EvalLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance);
	if (EvalLog::curr_inst_remaining_budget <= 0) {
		EvalLog::curr_inst_remaining_budget = 0;
	}

	if (EvalLog::curr_inst_remaining_steps <= 0) {
		EvalLog::allocated_time = 0;
	} else {
		EvalLog::allocated_time = (EvalLog::curr_inst_remaining_budget - 2.0)
			/ EvalLog::curr_inst_remaining_steps;

		if (EvalLog::allocated_time > 5.0)
			EvalLog::allocated_time = 5.0;
	}
}

void IPPCEvaluator::UpdateTimePerMove(double step_time) {
	if (step_time < 0.99 * EvalLog::allocated_time) {
		if (EvalLog::plan_time_ratio < 1.0)
			EvalLog::plan_time_ratio += 0.01;
		if (EvalLog::plan_time_ratio > 1.0)
			EvalLog::plan_time_ratio = 1.0;
	} else if (step_time > EvalLog::allocated_time) {
		double delta = (step_time - EvalLog::allocated_time)
			/ (EvalLog::allocated_time + 1E-6);
		if (delta < 0.02)
			delta = 0.02; // Minimum reduction per step
		if (delta > 0.05)
			delta = 0.05; // Maximum reduction per step
		EvalLog::plan_time_ratio -= delta;
		// if (EvalLog::plan_time_ratio < 0)
		// EvalLog::plan_time_ratio = 0;
	}

	EvalLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance_);
	EvalLog::curr_inst_remaining_steps--;

	UpdateTimeInfo(instance_);
	Globals::config.time_per_move = EvalLog::plan_time_ratio
		* EvalLog::allocated_time;

	if (!Globals::config.silence && out_) {
		*out_
			<< "Total time: curr_inst / inst_target / remaining / since_start = "
			<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
			<< (EvalLog::curr_inst_target_time
				* (EvalLog::curr_inst_steps - EvalLog::curr_inst_remaining_steps))
			<< " / " << EvalLog::curr_inst_remaining_budget << " / "
			<< (get_time_second() - EvalLog::start_time) << endl;
	}
}

/* =============================================================================
 * POMDPEvaluator class
 * =============================================================================*/

POMDPEvaluator::POMDPEvaluator(DSPOMDP* model, string belief_type,
	Solver* solver, clock_t start_clockt, ostream* out,
	double target_finish_time, int num_steps) :
	Evaluator(model, belief_type, solver, start_clockt, out),
	random_((unsigned) 0) {
	target_finish_time_ = target_finish_time;

	if (target_finish_time_ != -1) {
		EvalLog::allocated_time = (target_finish_time_ - get_time_second())
			/ num_steps;
		Globals::config.time_per_move = EvalLog::allocated_time;
		EvalLog::curr_inst_remaining_steps = num_steps;
	}
}

POMDPEvaluator::~POMDPEvaluator() {
}

int POMDPEvaluator::Handshake(string instance) {
	return -1; // Not to be used
}

void POMDPEvaluator::InitRound() {
	step_ = 0;

	double start_t, end_t;
	// Initial state
	state_ = model_->CreateStartState();
	logi << "[POMDPEvaluator::InitRound] Created start state." << endl;
	if (!Globals::config.silence && out_) {
		*out_ << "Initial state: " << endl;
		model_->PrintState(*state_, *out_);
		*out_ << endl;
	}

	// Initial belief
	start_t = get_time_second();
	delete solver_->belief();
	end_t = get_time_second();
	logi << "[POMDPEvaluator::InitRound] Deleted old belief in "
		<< (end_t - start_t) << "s" << endl;

	start_t = get_time_second();
	Belief* belief = model_->InitialBelief(state_, belief_type_);
	end_t = get_time_second();
	logi << "[POMDPEvaluator::InitRound] Created intial belief "
		<< typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;

	solver_->belief(belief);

	total_discounted_reward_ = 0;
	total_undiscounted_reward_ = 0;
}

double POMDPEvaluator::EndRound() {
	if (!Globals::config.silence && out_) {
		*out_ << "Total discounted reward = " << total_discounted_reward_ << endl
			<< "Total undiscounted reward = " << total_undiscounted_reward_ << endl;
	}

	discounted_round_rewards_.push_back(total_discounted_reward_);
	undiscounted_round_rewards_.push_back(total_undiscounted_reward_);

	return total_undiscounted_reward_;
}

bool POMDPEvaluator::ExecuteAction(int sim_type, int real_state, int action, double& reward, OBS_TYPE& obs) {
	bool terminal = true;
	double random_num = random_.NextDouble();

	if (sim_type == 0){ // simulator generates random(relevant) next states
		terminal = model_->Step(*state_, random_num, action, reward, obs);
	}
	else if (sim_type == 1){ // TODO: autonomous human model --> controlled environment
		terminal = model_->Step(*state_, random_num, action, reward, obs);
	}
	else{ //manually controlled human model --> uncontrolled environment
		//TODO: observation has no value and has no effect in this StepManual func. To be removed
		terminal = model_->StepManual(*state_, random_num, real_state, action, reward, obs);
	}

	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;
	return terminal;
}

double POMDPEvaluator::End() {
	return 0; // Not to be used
}

void POMDPEvaluator::UpdateTimePerMove(double step_time) {
	if (target_finish_time_ != -1) {
		if (step_time < 0.99 * EvalLog::allocated_time) {
			if (EvalLog::plan_time_ratio < 1.0)
				EvalLog::plan_time_ratio += 0.01;
			if (EvalLog::plan_time_ratio > 1.0)
				EvalLog::plan_time_ratio = 1.0;
		} else if (step_time > EvalLog::allocated_time) {
			double delta = (step_time - EvalLog::allocated_time)
				/ (EvalLog::allocated_time + 1E-6);
			if (delta < 0.02)
				delta = 0.02; // Minimum reduction per step
			if (delta > 0.05)
				delta = 0.05; // Maximum reduction per step
			EvalLog::plan_time_ratio -= delta;
			// if (EvalLog::plan_time_ratio < 0)
			// EvalLog::plan_time_ratio = 0;
		}

		EvalLog::curr_inst_remaining_budget = target_finish_time_
			- get_time_second();
		EvalLog::curr_inst_remaining_steps--;

		if (EvalLog::curr_inst_remaining_steps <= 0) {
			EvalLog::allocated_time = 0;
		} else {
			EvalLog::allocated_time =
				(EvalLog::curr_inst_remaining_budget - 2.0)
					/ EvalLog::curr_inst_remaining_steps;

			if (EvalLog::allocated_time > 5.0)
				EvalLog::allocated_time = 5.0;
		}

		Globals::config.time_per_move = EvalLog::plan_time_ratio
			* EvalLog::allocated_time;
	}
}

} // namespace despot
