#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include <cassert>
#include <cmath>
#include <iostream>
#include <cfloat>
#include "Matrix.h"
#include "Simulator.hpp"
using namespace std;
using namespace mathtool;

class MySVM;

struct Control
{
	float k_rho;
	float k_alpha;
	float k_beta;

	// for learning
	float best_time;
	float dx;
	float dy;
	float dtheta;


	Control()
	{
		k_rho = 1.0;
		k_alpha = 0.02;
		k_beta = 0.02;
		best_time = FLT_MAX;

		dx=dy=dtheta=0;
	}

	Control(float kRho, float kAlpha, float kBeta)
	{
		k_rho = kRho;
		k_alpha = kAlpha;
		k_beta = kBeta;


		best_time = FLT_MAX;
		dx=dy=dtheta=0;
	}
};


struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};

    Vertex(const int parent, const State& state, const vector<State>& path, const double path_length, const double path_time)
    {
    	m_vid = -1;
    	m_parent = parent;
    	m_type = 0;
    	m_nchildren = 0;
    	m_neighbor = 0;
    	m_state = state;
    	m_path = path;

    	m_path_length = path_length;
    	m_path_time = path_time;
    }
	
    int	   	m_vid;
    int    	m_parent;
    State m_state;
    int    	m_type;
    int    	m_nchildren;
    int	   	m_neighbor;
    double  m_path_length;
    double  m_path_time;

    vector<State> m_path;

    double weight() {
    	return 2.0/(1+exp(2*m_neighbor));
    }
};

    

class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator, bool use_best_control, bool predict, bool constraint);
            
    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void findBestControls(void);

    void ExtendUnitCircle(void);
    
    double GettotalSolveTime() const { return m_totalSolveTime; }

    int GetTotalVertices() const { return m_vertices.size(); }

    bool IsProblemSolved(void) { return m_vidAtGoal >= 0; }

    void ExportPath(const string& filename) const;

    static inline double AngleDiff(double source, double target)
	{
		return atan2(sin(target-source), cos(target-source));
	}

    static double Dist(const double* st1, const vector<double>& st2);
    static double DistSE2(const State& st1, const State& st2);

    // dist from st1 to st2
    double Dist(const State& source, const State& target);

    const Vertex* GetVertex(int vid) { return this->m_vertices[vid]; }

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    bool getDynamicConstraint() const { return this->m_dynamic_constraint; }
    void setDyanmicConstraint(bool dc) { this->m_dynamic_constraint = dc;}

protected:

    vector<State> SimDiffDrive(const State& start, const double vel, const double omega, const int steps, const double delta);
    State SimDiffDriveOneStep(const State& start, const double vel, const double omega, const double delta);

    // find a path from start to goal
    // output parameters:
    // 	total_moved
    //  time_traveled
    vector<State> DiffDriveGoTo(const Vertex* start, const State& goal, const double max_expension_dist, const Control& control,  vector<double>& total_moved, vector<double>& time_traveled, bool& hit_obst);

    static inline double Limit(double val, double min_val, double max_val)
    {
    	assert(min_val <= max_val);

    	return min(max(val, min_val), max_val);
    }

    static inline double wrapAngle( double angle )
	{
		while(angle>=2*PI) angle -= 2*PI;
		while(angle<=-2*PI) angle += 2*PI;
		return angle;
	}




    void AddVertex(Vertex * const v);

    void GetGrid(Vertex* const v, int& x, int& y);

    Vertex* ExtendTree(const int    vid,
		    const double sto[], int max_steps = 0);
    

    // extend the tree from Vertex[vid] by applying v and omega for time t
    Vertex* ExtendTreeDiffDrive(const int vid, const double vel, const double omega, const double t);

    // extend the tree from Vertex[vid] to goal
    Vertex* ExtendTreeDiffDrive(const int vid, const State& goal, const double max_expension_dist, const Control& control);

    State RandomConfig();

    // generate an random control
    void RandomControl(double& vel, double& omega);

    void SaveBestControl();
    void LoadBestControl();

    int GetControlIndex(double dx, double dy, double dtheta);

    // trainning
    void trainSVM();
    void testSVM();




    Simulator            	*m_simulator;
    std::vector<Vertex *>	m_vertices;
    int                   	m_vidAtGoal;
    double                	m_totalSolveTime;
    vector<vector<int> >	m_grids;
    int						m_max_steps;
    int						m_max_steps2;

    // apply constraints
    bool					m_dynamic_constraint;


    // training
    bool					m_predict;
    bool					m_use_best_control;
    MySVM*					m_svm;
    Control*				m_best_control;
    int						m_train_examples;
    int						m_valid_examples;

    friend class Graphics;    
};

#endif
