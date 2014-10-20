#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include "Matrix.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;
using namespace mathtool;

const double P = 0.1;
const int NUM_GRIDS = 50;
static const double MAX_OMEGA = 10;
static const double MAX_VEL_ACC = 0.5;
static const double MAX_OMEGA_ACC = 5;
static const double MAX_VEL = 2.5;
static const double MAX_EXPENSION = 2.5;
static const double ANGLE_DIFF_FACTOR = 0.1;

MotionPlanner::MotionPlanner(Simulator * const simulator)
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

    printf("m_max_steps = %d m_max_steps2 = %d\n", m_max_steps, m_max_steps2);

    auto state = State(m_simulator->GetRobotCenterX(), m_simulator->GetRobotCenterY(), 0.0);

    auto path = vector<State>();

    auto vinit = new Vertex(-1,state, path);

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
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

Vertex* MotionPlanner::ExtendTreeDiffDrive(const int vid, const double vel, const double omega, const double t)
{
	// get resolution
	auto res = m_simulator->GetDistOneStep();
	auto gs = this->m_simulator->GetGoalState();
	auto v = m_vertices[vid];

	auto max_t = omega == 0 ? t : min(t, 2*PI/fabs(omega));

	auto delta = m_simulator->GetTimeOneStep();
	auto steps = max_t / delta;

	//auto curState = v->m_state;

	auto lastState = v->m_state;

	auto steps_moved = 0;

	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();

	auto states = this->SimDiffDrive(v->m_state, vel, omega, steps, delta);

	for(auto i=0;i<steps;i++)
	{
		const auto& curState = states[i];

		// config robot
		this->m_simulator->SetRobotCenter(curState.x, curState.y);

		if(!this->m_simulator->IsValidState())
		{
			if(i==0) return NULL;
			break;
		}

		lastState = curState;
		steps_moved = i+1;

		auto dist = this->DistSE2(curState, gs);

		if(dist < dist_threshold) break;
	}

	auto path = vector<State>(states.begin(), states.begin()+steps_moved);

	auto new_vertex = new Vertex(vid, lastState, path);

	auto dist = this->DistSE2(new_vertex->m_state, gs);

	// goal reached
	if(dist < dist_threshold)
	{
		new_vertex->m_type = new_vertex->TYPE_GOAL;
		cout<<"dist = "<<dist<<" goal reached!"<<endl;
	}

	this->AddVertex(new_vertex);

	return new_vertex;
}

Vertex* MotionPlanner::ExtendTreeDiffDrive(const int vid, const State& goal)
{
	// get resolution
	auto res = m_simulator->GetDistOneStep();
	auto gs = this->m_simulator->GetGoalState();
	auto v = m_vertices[vid];
	auto delta = m_simulator->GetTimeOneStep();

	auto lastState = v->m_state;

	auto steps_moved = 0;

	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();

	auto last_omega = 0.0;
	auto last_vel = 0.0;
	auto total_moved = 0.0;

	auto path = this->DiffDriveGoTo(v, goal, total_moved);

	if(path.size() == 0) return NULL;

	auto state = path.back();

	auto new_vertex = new Vertex(vid, state, path);

	auto dist = this->DistSE2(new_vertex->m_state, gs);

	// goal reached
	if(dist < dist_threshold)
	{
		new_vertex->m_type = new_vertex->TYPE_GOAL;
		cout<<"dist = "<<dist<<" goal reached!"<<endl;
	}

	this->AddVertex(new_vertex);

	return new_vertex;
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



vector<State> MotionPlanner::DiffDriveGoTo(const Vertex* start_v, const State& goal, double& total_moved)
{
	auto output = vector<State>();

	static auto const k_rho = 2.0;
	static auto const k_alpha = 0.02;
	static auto const k_beta = 0.5;

	const auto res = this->m_simulator->GetDistOneStep();
	const auto delta = this->m_simulator->GetTimeOneStep();
	const auto gs = this->m_simulator->GetGoalState();
	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();
	auto start = start_v->m_state;

	auto T = Matrix3x3(
			cos(goal.theta),	-sin(goal.theta), 	goal.x,
			sin(goal.theta), 	cos(goal.theta), 	goal.y,
			0,				0,				1		);

	auto goal_t = State();
	auto start_t_v = T.inv() * Vector3d(start.x, start.y, 1);
	auto start_t = State(start_t_v[0], start_t_v[1], start.theta-goal.theta, start.vel, start.omega);

	auto now_t = start_t;

	// init output parameters
	total_moved = 0.0;
	auto last_vel = start_v->m_state.vel;
	auto last_omega = start_v->m_state.omega;

	while(true)
	{
		auto diff_x = goal_t.x - now_t.x;
		auto diff_y = goal_t.y - now_t.y;

		auto rho = sqrt(diff_x*diff_x + diff_y*diff_y);
		auto alpha = -now_t.theta + atan2(diff_y, diff_x);
		auto beta = now_t.theta + alpha;

		auto vel =  min(k_rho*rho, MAX_VEL);

		vel = this->Limit(vel, last_vel-MAX_VEL_ACC*delta, last_vel+MAX_VEL_ACC*delta);

		auto omega = k_alpha * alpha + k_beta*beta;

		omega = this->Limit(omega, last_omega-MAX_OMEGA_ACC*delta, last_omega+MAX_OMEGA_ACC*delta);

		// [0,0,0] reached
		if (rho < res && this->AngleDiff(0, now_t.theta) < res*0.5)
		{
			break;
		}

		// move one step
		now_t = this->SimDiffDriveOneStep(now_t, vel, omega, delta);

		// back to workspace
		auto now = T*Vector3d(now_t.x, now_t.y, 1);
		now[2]= now_t.theta + goal.theta;

		auto now_state = State(now[0], now[1], now[2], vel, omega);

		this->m_simulator->SetRobotState(now_state);

		if(!this->m_simulator->IsValidState())
		{
			break;
		}

		last_vel = vel;
		last_omega = omega;

		output.push_back(now_state);

		auto dist = this->DistSE2(now_state, gs);

		// goal reached
		if(dist < dist_threshold)
		{
			break;
		}

		total_moved += vel*delta;

		if(total_moved > MAX_EXPENSION) break;
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

	this->ExtendTreeDiffDrive(vid, cfg);

    
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
    	double dist = this->DistSE2(cfg, v->m_state);
    	if(dist < min_dist)
    	{
    		min_dist = dist;
    		closest = v;
    		vid = i;
    	}
    }

    if(closest)
    {
    	this->ExtendTreeDiffDrive(vid, cfg);
    }

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
//    Clock clk;
//    StartTime(&clk);
//
//
//	Vertex* selected = NULL;
//	int vid = -1;
//	double total_weight = 0;
//	int size = this->m_vertices.size();
//
//	const double delta = 2*this->m_simulator->GetDistOneStep();
//
//	// count neighbors
//	for(int i=0;i<size;++i)
//	{
//		Vertex* v = this->m_vertices[i];
//		int x,y;
//		this->GetGrid(v, x, y);
//		v->m_neighbor = this->m_grids[y][x];
//		total_weight += v->weight();
//	}
//
//	double w = PseudoRandomUniformReal() * total_weight;
//	// in reverse order, newly added nodes have more probability to be selected
//	for(int i=size-1;i>=0;i--)
//	{
//		Vertex* v = this->m_vertices[i];
//		if( v->weight() >= w)
//		{
//			selected = v;
//			vid = i;
//			break;
//		}
//
//		w -= v->weight();
//	}
//
//	if(selected)
//	{
//	    // generate a random config
//	    double sto[2];
//
//	    while(true)
//	    {
//	    	this->RandomConfig(sto);
//	    	// q_rand should near to selected
//	    	if(this->Dist(sto, selected->m_state) < 20 * this->m_simulator->GetDistOneStep()) break;
//	    }
//
//
//		this->ExtendTree(vid, sto);
//	}
//
//    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
//    Clock clk;
//    StartTime(&clk);
//
//    // generate a random config
//	double sto[2];
//	this->RandomConfig(sto);
//
//	double min_dist = 1e10;
//	Vertex* closest = NULL;
//	int vid = -1;
//
//	// find the closest vertex in the tree
//	for(int i=0, size = this->m_vertices.size();i<size;++i)
//	{
//		Vertex* v = this->m_vertices[i];
//		double dist = this->Dist(sto, v->m_state);
//		if(dist < min_dist)
//		{
//			min_dist = dist;
//			closest = v;
//			vid = i;
//		}
//	}
//
//	if(closest)
//	{
//		Vertex* last_added = this->ExtendTree(vid, sto, this->m_max_steps);
//
//		if(last_added && m_vidAtGoal < 0)
//		{
//			double v1[2] = {last_added->m_state[0] - closest->m_state[0], last_added->m_state[1] - closest->m_state[1]};
//			double v2[2] = {-v1[1]*1000, v1[0]*1000};
//			double d1[2] = {last_added->m_state[0] + v2[0], last_added->m_state[1] + v2[1]};
//			double d2[2] = {last_added->m_state[0] - v2[0], last_added->m_state[1] - v2[1]};
//
//			this->ExtendTree(last_added->m_vid, d1, m_max_steps2);
//			this->ExtendTree(last_added->m_vid, d2, m_max_steps2);
//		}
//	}
//
//    m_totalSolveTime += ElapsedTime(&clk);
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
			fout<<traj.x<<" "<<traj.y<<" "<<traj.theta<<" "<<traj.vel<<" "<<traj.omega<<endl;
		}
	}

	fout.close();
}
