#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"
#include <cmath>
#include <iostream>
using namespace std;

struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};

    Vertex(const int parent, const int steps, const double vel, const double omega, const vector<double>& state, const vector<vector<double> > path)
    {
    	m_vid = -1;
    	m_parent = parent;
    	m_type = 0;
    	m_nchildren = 0;
    	m_neighbor = 0;
    	m_state = state;

    	// control
    	m_vel = vel;
    	m_omega = omega;
    	m_steps = steps;

    	m_path = path;
    }
	
    int	   	m_vid;
    int    	m_parent;
    vector<double> m_state;
    int    	m_type;
    int    	m_nchildren;
    int	   	m_neighbor;
    int		m_steps;
    double 	m_vel;
    double 	m_omega;

    vector<vector<double> > m_path;

    double weight() {
    	return 2.0/(1+exp(2*m_neighbor));
    }
};

    

class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator);
            
    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void ExtendEST(void);

    void ExtendMyApproach(void);
    
    double GettotalSolveTime() const { return m_totalSolveTime; }

    int GetTotalVertices() const { return m_vertices.size(); }

    bool IsProblemSolved(void) { return m_vidAtGoal >= 0; }

    static inline double wrapAngle( double angle )
	{
		double twoPi = 2.0 * 3.141592865358979;
		return angle - twoPi * floor( angle / twoPi );
	}

    vector<vector<double> > SimDiffDrive(const vector<double>& start, const double vel, const double omega, const int steps, const double delta);

protected:

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    void GetGrid(Vertex* const v, int& x, int& y);

    Vertex* ExtendTree(const int    vid,
		    const double sto[], int max_steps = 0);
    

    // extend the tree from Vertex[vid] by applying v and omega for time t
    Vertex* ExtendTreeDiffDrive(const int vid, const double vel, const double omega, const double t);

    vector<double> RandomConfig();

    // generate an random control
    void RandomControl(double& vel, double& omega);

    double Dist(const double* st1, const vector<double>& st2);
    double DistSE2(const vector<double>& st1, const vector<double>& st2);

    Simulator            	*m_simulator;
    std::vector<Vertex *>	m_vertices;
    int                   	m_vidAtGoal;
    double                	m_totalSolveTime;
    vector<vector<int> >	m_grids;
    int						m_max_steps;
    int						m_max_steps2;
    
    friend class Graphics;    
};

#endif
