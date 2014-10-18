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
	
    int	   m_vid;
    int    m_parent;
    double m_state[Simulator::STATE_NR_DIMS];
    int    m_type;
    int    m_nchildren;
    int	   m_neighbor;
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
        
protected:

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    void GetGrid(Vertex* const v, int& x, int& y);

    Vertex* ExtendTree(const int    vid,
		    const double sto[], int max_steps = 0);
    
    void RandomConfig(double cfg[]);

    double Dist(const double* st1, const double* st2);

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
