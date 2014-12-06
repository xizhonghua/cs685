#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include "Matrix.h"
#include "MySVM.h"

using namespace std;
using namespace mathtool;

const double P = 0.05;
const int NUM_GRIDS = 50;
static const double MAX_VEL = 1;
static const double MAX_VEL_ACC = 0.2;
static const double MAX_OMEGA = 1;
static const double MAX_OMEGA_ACC = 1;
static const double MAX_EXPENSION = 10;
static const double ANGLE_DIFF_FACTOR = 0.5;

static const Control DEFAULT_CTL = Control(0.5, 1, 1);


static const int X_BLOCKS = 21;
static const int Y_BLOCKS = 21;
static const int THETA_BLOCKS = 37;

vector<Example> examples;

MotionPlanner::MotionPlanner(Simulator * const simulator, bool predict)
{
    m_simulator = simulator;   

    for(int i=0;i<NUM_GRIDS;i++)
	{
		vector<int> grid(NUM_GRIDS);
		this->m_grids.push_back(grid);
	}

    const double* bbox = m_simulator->GetBoundingBox();
    double max_length = sqrt((bbox[2] - bbox[0])*(bbox[2] - bbox[0]) + (bbox[3] - bbox[1])*(bbox[3] - bbox[1]));

    // 10 %
    double const max_movement = 0.2;
    double const max_movement2 = 0.1;

    this->m_max_steps = max_length * max_movement / this->m_simulator->GetDistOneStep();

    // 20 % of max_steps
    this->m_max_steps2 = std::max(1, (int)(max_movement2 * max_length * max_movement / this->m_simulator->GetDistOneStep()));

    fprintf(stderr, "m_max_steps = %d m_max_steps2 = %d\n", m_max_steps, m_max_steps2);

    auto state = State(m_simulator->GetRobotCenterX(), m_simulator->GetRobotCenterY(), 0.0);

    auto path = vector<State>();

    auto vinit = new Vertex(-1,state, path, 0, 0);

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;

    this->m_predict = predict;

    this->m_train_examples = 0;
    this->m_valid_examples = 0;

    this->m_best_control = NULL;

    if(this->m_predict)
    {
//    	this->m_svm = new MySVM();
//    	this->m_svm->LoadModel();
    	this->LoadBestControl();
    }
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}

double MotionPlanner::Dist(const double* st1, const vector<double>& st2)
{
	return sqrt((st1[0]-st2[0])*(st1[0]-st2[0]) + (st1[1]-st2[1])*(st1[1]-st2[1]));
}

double MotionPlanner::DistSE2(const State& st1, const State& st2)
{
	auto d1 = sqrt((st1.x-st2.x)*(st1.x-st2.x) + (st1.y - st2.y)*(st1.y-st2.y));

	auto d2 = ANGLE_DIFF_FACTOR*min(fabs(st1.theta - st2.theta), 2*PI-fabs(st1.theta - st2.theta));

	return sqrt(d1*d1 + d2*d2);
}

Vertex* MotionPlanner::ExtendTreeDiffDrive(const int vid, const State& goal, const double max_expension_dist, const Control& control)
{
	// get resolution
	auto res = m_simulator->GetDistOneStep();
	auto gs = this->m_simulator->GetGoalState();
	auto v = m_vertices[vid];
	auto delta = m_simulator->GetTimeOneStep();

	auto lastState = v->m_state;

	auto steps_moved = 0;

	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();

	auto total_moved = vector<double>();
	auto time_traveled = vector<double>();
	bool hit_obst;

	auto path = this->DiffDriveGoTo(v, goal, max_expension_dist, control, total_moved, time_traveled, hit_obst);

	if(path.size() == 0) return NULL;

//	cout<<"goal = "<<goal<<" reached = "<<path.back()<<endl;

	if(hit_obst && path.size()>3)
	{
		auto half_size = path.size()/2;
		auto first_half_path = vector<State>(path.begin(), path.begin()+half_size);
		auto second_half_path = vector<State>(path.begin()+half_size+1, path.end());

		auto first_half_path_length = total_moved[half_size-1];
		auto second_half_path_length = total_moved.back() - first_half_path_length;

		auto first_half_path_time = time_traveled[half_size-1];
		auto second_half_path_time = time_traveled.back() - first_half_path_time;

		auto mid_state = path[half_size-1];
		auto end_state = path.back();

		auto mid_vertex = new Vertex(vid, mid_state, first_half_path, first_half_path_length, first_half_path_time);

		this->AddVertex(mid_vertex);

		auto end_vertex = new Vertex(mid_vertex->m_vid, end_state, second_half_path, second_half_path_length, second_half_path_time);

		this->AddVertex(end_vertex);

		auto dist = this->DistSE2(end_vertex->m_state, gs);

		// goal reached
		if(dist < dist_threshold)
		{
			end_vertex->m_type = end_vertex->TYPE_GOAL;
		}

		return end_vertex;

	}
	else
	{

		auto state = path.back();

		auto new_vertex = new Vertex(vid, state, path, total_moved.back(), time_traveled.back());

		auto dist = this->DistSE2(new_vertex->m_state, gs);

		// goal reached
		if(dist < dist_threshold)
		{
			new_vertex->m_type = new_vertex->TYPE_GOAL;
		}

		this->AddVertex(new_vertex);

		return new_vertex;
	}
}

State MotionPlanner::SimDiffDriveOneStep(const State& start, const double vel, const double omega, const double delta)
{
	auto curState = start;

	curState.vel = vel;
	curState.omega = omega;

	curState.x += vel*cos(curState.theta)*delta;
	curState.y += vel*sin(curState.theta)*delta;
	curState.theta += omega*delta;
	curState.theta = this->wrapAngle(curState.theta);

	return curState;
}

vector<State> MotionPlanner::SimDiffDrive(const State& start, const double vel, const double omega, const int steps, const double delta)
{
	auto curState = start;
	auto output = vector<State>();

	for(auto i=0;i<steps;i++)
	{
		curState = this->SimDiffDriveOneStep(curState, vel, omega, delta);
		output.push_back(curState);
	}

	return output;
}


vector<State> MotionPlanner::DiffDriveGoTo(const Vertex* start_v, const State& goal, const double max_expension_dist, const Control& control, vector<double>& total_moved, vector<double>& time_traveled, bool& hit_obst)
{
	auto output = vector<State>();

	const auto res = this->m_simulator->GetDistOneStep();
	const auto delta = 0.1;
	const auto& gs = this->m_simulator->GetGoalState();
	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();
	auto start = start_v->m_state;

	auto T = Matrix3x3(
			cos(goal.theta),	-sin(goal.theta), 	goal.x,
			sin(goal.theta), 	cos(goal.theta), 	goal.y,
			0,				0,				1		);

	auto goal_t = State();
	auto start_t_v = T.inv() * Vector3d(start.x, start.y, 1);
	auto start_t = State(start_t_v[0], start_t_v[1], start.theta-goal.theta, start.vel, start.omega, start.clock);

	auto now_t = start_t;

//	cerr<<"goal "<<goal<<" start"<<start<<endl;
//	cerr<<"goal_t"<<goal_t<<" start_t"<<start_t<<endl;

	// init output parameters
	auto moved = 0.0;
	auto time_used = 0.0;

	hit_obst = false;
	total_moved.clear();
	time_traveled.clear();


	auto last_vel = start_v->m_state.vel;
	auto last_omega = start_v->m_state.omega;

	auto steps = 0;

	while(++steps)
	{
		auto diff_x = goal_t.x - now_t.x;
		auto diff_y = goal_t.y - now_t.y;
		auto delta_theta = goal_t.theta - now_t.theta;

		auto rho = sqrt(diff_x*diff_x + diff_y*diff_y);
		auto alpha = -now_t.theta + atan2(diff_y, diff_x);
		auto beta = now_t.theta + alpha;

		auto vel = control.k_rho*rho;

		auto omega = control.k_alpha * alpha + control.k_beta*beta;

		// apply dynamic constraint
		if(this->m_dynamic_constraint)
		{
			vel = this->Limit(vel, last_vel-MAX_VEL_ACC*delta, last_vel+MAX_VEL_ACC*delta);

			vel = this->Limit(vel, -MAX_VEL, MAX_VEL);

			omega = this->Limit(omega, last_omega-MAX_OMEGA_ACC*delta, last_omega+MAX_OMEGA_ACC*delta);

			omega = this->Limit(omega, -MAX_OMEGA, MAX_OMEGA);
		}

		// move one step
		now_t = this->SimDiffDriveOneStep(now_t, vel, omega, delta);

		// back to workspace
		auto now = T*Vector3d(now_t.x, now_t.y, 1);
		now[2]= now_t.theta + goal.theta;

		auto now_state = State(now[0], now[1], now[2], vel, omega, start.clock + steps*delta);

//		fprintf(stderr, "rho = %f delta_theta = %f alpha = %f beta = %f v = %f omega = %f\n",rho, delta_theta, alpha, beta, vel, omega);
//		cerr<<"goal_t = "<<goal_t<<" now_t = "<<now_t<<endl;
//		cerr<<"goal = "<<goal<<" now = "<<now_state<<endl;
//		cerr<<string(60,'-')<<endl;


		this->m_simulator->SetRobotState(now_state);


		if(!this->m_simulator->IsValidState())
		{
			hit_obst = true;
			break;
		}

		// [0,0,0] reached
		if (rho < res && fabs(this->AngleDiff(0, now_t.theta)) < res*2)
		{
			break;
		}

		last_vel = vel;
		last_omega = omega;

		output.push_back(now_state);

		moved += fabs(vel)*delta;

		total_moved.push_back(moved);

		time_traveled.push_back(output.size()*delta);

		if(moved > max_expension_dist || steps>10000) break;

		auto dist = this->DistSE2(now_state, gs);

		// goal reached
		if(dist < dist_threshold)
		{
			break;
		}
	}

	return output;
}


State MotionPlanner::RandomConfig()
{
	if(PseudoRandomUniformReal() < P)
	{
		// select random state from goal region with probability P
		return this->m_simulator->GetGoalState();
	}
	else
	{
	    // select random state uniformly with probability = (1 - P)
		return this->m_simulator->SampleState();
	}
}

void MotionPlanner::RandomControl(double& vel, double& omega)
{
	vel = PseudoRandomUniformReal() < 0.5 ? MAX_VEL : - MAX_VEL;
	omega = PseudoRandomUniformReal()*MAX_OMEGA*2 - MAX_OMEGA;
}


void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

	auto vid = PseudoRandomUniformReal() * this->m_vertices.size();

	auto cfg = this->RandomConfig();

	this->ExtendTreeDiffDrive(vid, cfg, MAX_EXPENSION, Control());

    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);

    // generate a random config

    auto cfg = this->RandomConfig();

    auto vel = 0.0;
    auto omega = 0.0;

    this->RandomControl(vel, omega);

    double min_dist = 1e10;
    Vertex* closest = NULL;
    int vid = -1;

    // find the closest vertex in the tree
    for(int i=0, size = this->m_vertices.size();i<size;++i)
    {
    	Vertex* v = this->m_vertices[i];
    	double dist = this->Dist(v->m_state, cfg);
    	if(dist < min_dist)
    	{
    		min_dist = dist;
    		closest = v;
    		vid = i;
    	}
    }

    if(closest)
    {
    	// use default control
    	auto ctl = DEFAULT_CTL;

    	if(m_predict)
    	{
    		// get learned control
    		auto dx = cfg.x - closest->m_state.x;
    		auto dy = cfg.y - closest->m_state.y;
    		auto dtheta = this->AngleDiff(closest->m_state.theta, cfg.theta);

    		auto idx = this->GetControlIndex(dx, dy, dtheta);

    		ctl = this->m_best_control[idx];
    	}

    	this->ExtendTreeDiffDrive(vid, cfg, MAX_EXPENSION, ctl);
    }

    m_totalSolveTime += ElapsedTime(&clk);
}

int MotionPlanner::GetControlIndex(double dx, double dy, double dtheta)
{
	if(dx < -MAX_EXPENSION) dx = -MAX_EXPENSION;
	if(dx > MAX_EXPENSION) dx = MAX_EXPENSION;

	if(dy < -MAX_EXPENSION) dy = -MAX_EXPENSION;
	if(dy > MAX_EXPENSION) dy = MAX_EXPENSION;

	auto x_id = mathtool::round(dx / MAX_EXPENSION * X_BLOCKS);
	auto y_id = mathtool::round(dy / MAX_EXPENSION * Y_BLOCKS);
	auto theta_id = mathtool::round(dtheta / PI * THETA_BLOCKS);

	auto idx = (x_id+X_BLOCKS) + (y_id + Y_BLOCKS)*(X_BLOCKS*2+1) + (theta_id + THETA_BLOCKS)*(X_BLOCKS*2+1)*(Y_BLOCKS*2+1);

	return idx;
}


void MotionPlanner::ExtendTrain(void)
{
	if(!this->m_best_control)
	{
		this->m_best_control = new Control[(X_BLOCKS*2+1)*(Y_BLOCKS*2+1)*(THETA_BLOCKS*2+1)];
		m_train_examples = 0;
	}

	auto theta1 = PseudoRandomUniformReal()*2*PI;
	auto theta2 = PseudoRandomUniformReal()*2*PI - PI;

	auto x = MAX_EXPENSION*(PseudoRandomUniformReal()*2 - 1);	// - MAX_EXPENSION ~ + MAX_EXPENSION
	auto y = MAX_EXPENSION*(PseudoRandomUniformReal()*2 - 1);	// - MAX_EXPENSION ~ + MAX_EXPENSION

	auto s = State(x, y, theta2);

	auto v0 = this->m_vertices[0];

	auto& v0_s = v0->m_state;
	v0_s.theta = 0.0;
	v0_s.vel = MAX_VEL;
	v0_s.omega = 0;

	auto ctl = Control();

	ctl.k_rho = PseudoRandomUniformReal()*5 + 0.2;
	ctl.k_alpha = PseudoRandomUniformReal()*0.2 - 0.1;
	ctl.k_beta = PseudoRandomUniformReal()*0.2 - 0.1;

	auto length = vector<double>();
	auto time = vector<double>();
	bool hit_obst;

	auto path = this->DiffDriveGoTo(v0, s, MAX_EXPENSION*5, ctl, length, time, hit_obst);

	if(path.size() == 0) return;

	const auto& vnew_s = path.back();

	auto dx = vnew_s.x - v0_s.x;
	auto dy = vnew_s.y - v0_s.y;
	auto dtheta = this->AngleDiff(v0_s.theta, vnew_s.theta);
	auto dvel = vnew_s.vel - v0_s.vel;
	auto domega = vnew_s.omega - v0_s.omega;
//	auto time = vnew->m_path_time;
//	auto length = vnew->m_path_length;

	ctl.best = time.back();

	auto st_time = sqrt(dx*dx + dy*dy) / MAX_VEL;

	auto idx = this->GetControlIndex(dx, dy, dtheta);

//	assert(idx>=0);
//	assert(idx<X_BLOCKS*Y_BLOCKS*THETA_BLOCKS*8);

	const auto& cur_ctl = this->m_best_control[idx];

	if(cur_ctl.best > ctl.best)
	{
	//	cout<<"idx = "<<idx<<" updated from "<<cur_ctl.best<<" to "<<ctl.best<<endl;
		this->m_best_control[idx] = ctl;
		++m_valid_examples;
	}

	++m_train_examples;

	if(m_train_examples % 10000 == 0)
	{
		cout<<" training "<<m_valid_examples<<"/"<<m_train_examples<<endl;
	}


	if(m_train_examples >= 1e6)
	{
		cout<<" - training finished!";
		this->SaveBestControl();
		exit(0);
	}
}


void MotionPlanner::ExtendUnitCircle(void)
{

	const int steps = 12;
	for(int i=0;i<steps;i++)
	{
		auto per = (double)i/steps;
		auto theta1 = per*2*PI;
		auto theta2 = theta1;

		auto x = cos(theta1)*MAX_EXPENSION;
		auto y = sin(theta1)*MAX_EXPENSION;

		auto goal_state = State(x, y, theta2);

		auto v0 = this->m_vertices[0];

		auto& v0_s = v0->m_state;
		v0_s.x = 0.0;
		v0_s.y = 0.0;
		v0_s.theta = 0.0;
		v0_s.vel = 0.0;
		v0_s.omega = 0.0;

		auto vnew = this->ExtendTreeDiffDrive(0, goal_state, MAX_EXPENSION*5, DEFAULT_CTL);

		if(!vnew) continue;


		if(this->DistSE2(goal_state, vnew->m_state) > this->m_simulator->GetDistOneStep()) continue;

		auto& vnew_s = vnew->m_state;

		auto dx = vnew_s.x - v0_s.x;
		auto dy = vnew_s.y - v0_s.y;
		auto dtheta = this->AngleDiff(v0_s.theta, vnew_s.theta);
		auto dvel = vnew_s.vel - v0_s.vel;
		auto domega = vnew_s.omega - v0_s.omega;
		auto time = vnew->m_path_time;
		auto length = vnew->m_path_length;

		auto st_time = sqrt(dx*dx + dy*dy) / MAX_VEL;

		cout<<dx<<" "<<dy<<" "<<dtheta<<" "<<time<<" "<<st_time<<" "<<time/st_time<<endl;


	}
}

void MotionPlanner::GetGrid(Vertex* const v, int& x, int& y)
{
	const double* bbox = m_simulator->GetBoundingBox();

	x = (v->m_state.x - bbox[0]) / (bbox[2] - bbox[0]) * NUM_GRIDS;
	y = (v->m_state.y - bbox[1]) / (bbox[3] - bbox[1]) * NUM_GRIDS;
}

void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
    	m_vidAtGoal = m_vertices.size();

    v->m_vid = this->m_vertices.size();

    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);

    int x,y;

    this->GetGrid(v, x, y);

    this->m_grids[y][x]++;
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}

void MotionPlanner::ExportPath(const string& filename) const
{
	ofstream fout;

	fout.open(filename);

	auto path = vector<int>();
	this->GetPathFromInitToGoal(&path);

	for(auto vid : path)
	{
		auto v = this->m_vertices[vid];

		for(auto traj : v->m_path)
		{
			fout<<traj<<endl;
		}
	}

	fout.close();
}


double MotionPlanner::Dist(const State& source, const State& target)
{
	auto se2_dist = MotionPlanner::DistSE2(source, target);

//	if(se2_dist > 3*MAX_EXPENSION) return se2_dist;
//
//	if(this->m_predict) return this->m_svm->Predict(source, target);

	return se2_dist;
}


void MotionPlanner::SaveBestControl()
{
	if(!this->m_best_control){
		cout<<" best control is NULL";
		exit(-1);
	}

	auto size = (X_BLOCKS*2+1)*(Y_BLOCKS*2+1)*(THETA_BLOCKS*2+1);

	ofstream fout("ctl.model");


	for(auto i=0;i<size;i++)
	{
		const auto& c = this->m_best_control[i];
		fout<<c.k_rho<<" "<<c.k_alpha<<" "<<c.k_beta<<endl;
	}

	fout.close();

	cout<<" - best control saved!";
}

void MotionPlanner::LoadBestControl()
{

	cerr<<" - Loading Best Control"<<endl;

	if(this->m_best_control)
	{
		delete m_best_control;
	}

	auto size = (X_BLOCKS*2+1)*(Y_BLOCKS*2+1)*(THETA_BLOCKS*2+1);

	this->m_best_control = new Control[size];

	ifstream fin("ctl.model");

	if(fin.bad())
	{
		cerr<<" ! failed to open ctl.model";
		return;
	}

	for(auto i=0;i<size;i++)
	{
		auto& c = this->m_best_control[i];
		fin>>c.k_rho>>c.k_alpha>>c.k_beta;
	}

	fin.close();
}
