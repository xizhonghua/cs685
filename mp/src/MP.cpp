#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
using namespace std;

const double P = 0.1;
const double PI = 3.1415926;
const int NUM_GRIDS = 50;
static const double MAX_OMEGA = 1.0;

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

    auto state = {m_simulator->GetRobotCenterX(), m_simulator->GetRobotCenterY(), 0.0};

    auto path = vector<vector<double> >();

    auto vinit = new Vertex(-1, 0, 0, 0, state, path);

//    vinit->m_parent   = -1;
//    vinit->m_nchildren= 0;
//    vinit->m_state[0] = m_simulator->GetRobotCenterX();
//    vinit->m_state[1] = m_simulator->GetRobotCenterY();

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

double MotionPlanner::DistSE2(const vector<double>& st1, const vector<double>& st2)
{
	auto d1 = sqrt((st1[0]-st2[0])*(st1[0]-st2[0]) + (st1[1]-st2[1])*(st1[1]-st2[1]));

	auto d2 = min(fabs(st1[2] - st2[2]), 2*PI-fabs(st1[2] - st2[2]));

	return sqrt(d1*d1 + d2*d2);
}

Vertex* MotionPlanner::ExtendTree(const int    vid,
			       const double sto[], int max_steps)
{
//	// get the vertex to expend from
//	Vertex* v = m_vertices[vid];
//	// get resolution
//	double res = m_simulator->GetDistOneStep();
//	// compute diff
//	double diff[2] = { sto[0] - v->m_state[0], sto[1] - v->m_state[1] };
//	// compute dist
//	double dist = this->Dist(v->m_state, sto);
//	// compute max steps
//	int steps = dist / res;
//
//	Vertex * last_added = NULL;
//
//	if(steps == 0) {
//		return NULL;
//	}
//
//	// forwarding vector
//	double ss[2] = { diff[0]/steps, diff[1]/steps };
//	// goal
//	double gs[2] = { this->m_simulator->GetGoalCenterX(), this->m_simulator->GetGoalCenterY() };
//
//	if(max_steps!=0)
//	{
//		steps = std::min(steps, max_steps);
//	}
//
//	for(int i=1;i<=steps;i++)
//	{
//		// temp state
//		double ts[2] = { v->m_state[0] + i*ss[0], v->m_state[1] + i*ss[1] };
//
//		// config robot
//		this->m_simulator->SetRobotCenter(ts[0], ts[1]);
//
//		// validity check
//		if(this->m_simulator->IsValidState())
//		{
//			Vertex *nv = new Vertex();
//
//			nv->m_parent = (i == 1 ? vid : this->m_vertices.size()-1);
//			nv->m_state[0] = ts[0];
//			nv->m_state[1] = ts[1];
//			nv->m_nchildren= 0;
//
//			double dist = this->Dist(ts, gs);
//
//			// close to goal
//			if(dist < res + this->m_simulator->GetGoalRadius())
//			{
//				nv->m_type = nv->TYPE_GOAL;
//			}
//
//			// add to tree
//			this->AddVertex(nv);
//
//			last_added = nv;
//		}
//		else
//		{
//			// hit obstacle return
//			return last_added;
//		}
//	}
//
//	return NULL;
}

Vertex* MotionPlanner::ExtendTreeDiffDrive(const int vid, const double vel, const double omega, const double t)
{
	// get resolution
	auto res = m_simulator->GetDistOneStep();
	auto gs = this->m_simulator->GetGoalState();
	auto v = m_vertices[vid];

	auto delta = m_simulator->GetTimeOneStep();
	auto steps = t / delta;

	//auto curState = v->m_state;

	auto lastState = v->m_state;

	auto steps_moved = 0;

	auto dist_threshold =  res + this->m_simulator->GetGoalRadius();

	auto states = this->SimDiffDrive(v->m_state, vel, omega, steps, delta);

	for(auto i=0;i<steps;i++)
	{
		const auto& curState = states[i];

		// config robot
		this->m_simulator->SetRobotCenter(curState[0], curState[1]);

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

	auto path = vector<vector<double> >(states.begin(), states.begin()+steps_moved);

	auto new_vertex = new Vertex(vid, steps_moved, vel, omega, lastState, path);

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

vector<vector<double> > MotionPlanner::SimDiffDrive(const vector<double>& start, const double vel, const double omega, const int steps, const double delta)
{
	auto curState = start;
	auto output = vector<vector<double> >();


	//cout<<"start = "<<start[0]<<" "<<start[1]<<" "<<start[2]<<endl;
	//cout<<"vel = "<<vel<<" omega = "<<omega<<" steps = "<<steps<<" delta = "<<delta<<endl;
	for(auto i=0;i<steps;i++)
	{
		curState[0] += vel*cos(curState[2])*delta;
		curState[1] += vel*sin(curState[2])*delta;
		curState[2] += omega*delta;
		curState[2] = this->wrapAngle(curState[2]);

		//cout<<"step "<<i<<" cur = "<<curState[0]<<" "<<curState[1]<<" "<<curState[2]<<endl;

		output.push_back(curState);
	}

	//cin>>curState[0];

	return output;
}


void MotionPlanner::RandomConfig(double cfg[])
{

	if(PseudoRandomUniformReal() < P)
	{
		// select random state from goal region with probability P
		cfg[0] =  this->m_simulator->GetGoalCenterX();
		cfg[1] = this->m_simulator->GetGoalCenterY();
	}
	else
	{
	    // select random state uniformly with probability = (1 - P)
		this->m_simulator->SampleState(cfg);
	}
}

void MotionPlanner::RandomControl(double& vel, double& omega)
{
	vel = 2;
	omega = PseudoRandomUniformReal()*MAX_OMEGA*2 - MAX_OMEGA;
}


void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);

	auto vid = PseudoRandomUniformReal() * this->m_vertices.size();

//	double sto[2];
//
//	this->RandomConfig(sto);
//
//    this->ExtendTree(vid, sto);

	auto vel = 0.0;
	auto omega = 0.0;

	this->RandomControl(vel, omega);

	this->ExtendTreeDiffDrive(vid, vel, omega, this->m_simulator->GetTimeOneStep()*100);
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
//    Clock clk;
//    StartTime(&clk);
//
//    // generate a random config
//    double sto[2];
//    this->RandomConfig(sto);
//
//    double min_dist = 1e10;
//    Vertex* closest = NULL;
//    int vid = -1;
//
//    // find the closest vertex in the tree
//    for(int i=0, size = this->m_vertices.size();i<size;++i)
//    {
//    	Vertex* v = this->m_vertices[i];
//    	double dist = this->Dist(sto, v->m_state);
//    	if(dist < min_dist)
//    	{
//    		min_dist = dist;
//    		closest = v;
//    		vid = i;
//    	}
//    }
//
//    if(closest)
//    {
//    	this->ExtendTree(vid, sto);
//    }
//
//    m_totalSolveTime += ElapsedTime(&clk);
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

	x = (v->m_state[0] - bbox[0]) / (bbox[2] - bbox[0]) * NUM_GRIDS;
	y = (v->m_state[1] - bbox[1]) / (bbox[3] - bbox[1]) * NUM_GRIDS;
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
