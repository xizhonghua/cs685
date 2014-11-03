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
	double k_rho;
	double k_alpha;
	double k_beta;
	double best;

	Control()
	{
		k_rho = 2.0;
		k_alpha = 0.02;
		k_beta = 0.2;
		best = FLT_MAX;
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
    MotionPlanner(Simulator * const simulator, bool predict);
            
    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void ExtendTrain(void);

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

protected:

    vector<State> SimDiffDrive(const State& start, const double vel, const double omega, const int steps, const double delta);
    State SimDiffDriveOneStep(const State& start, const double vel, const double omega, const double delta);

    // find a path from start to goal
    // output parameters:
    // 	total_moved
    //  time_traveled
    vector<State> DiffDriveGoTo(const Vertex* start, const State& goal, const double max_expension_dist, const Control& control,  vector<double>& total_moved, vector<double>& time_traveled);

    static inline double Limit(double val, double min_val, double max_val)
    {
    	assert(min_val <= max_val);

    	return min(max(val, min_val), max_val);
    }

    static inline double wrapAngle( double angle )
	{
		double twoPi = 2.0 * 3.141592865358979;
		return angle - twoPi * floor( angle / twoPi );
	}



    void GetPathFromInitToGoal(std::vector<int> *path) const;

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


    Simulator            	*m_simulator;
    std::vector<Vertex *>	m_vertices;
    int                   	m_vidAtGoal;
    double                	m_totalSolveTime;
    vector<vector<int> >	m_grids;
    int						m_max_steps;
    int						m_max_steps2;

    // training
    bool					m_predict;
    MySVM*					m_svm;
    Control*				m_best_control;
    int						m_train_examples;
    int						m_valid_examples;

    friend class Graphics;    
};

#endif
