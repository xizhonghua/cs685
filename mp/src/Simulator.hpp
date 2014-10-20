/**
 *@file Simulator.hpp
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "PseudoRandom.hpp"
#include "Vector.h"
using namespace mathtool;

struct State
{
	double x;
	double y;
	double theta;

	double vel;
	double omega;

	State()
	{
		x = 0;
		y = 0;
		theta = 0;
		vel = 0;
		omega = 0;
	}

	State(double x, double y, double theta):x(x),y(y),theta(theta)
	{
		vel = 0;
		omega = 0;
	}

	// constructor
	State(double x, double y, double theta, double vel, double omega) : x(x), y(y), theta(theta), vel(vel), omega(omega)
	{
		//nothing to do here
	}
};

class Simulator
{
public:    
    Simulator(void);
    
    ~Simulator(void);

    enum
	{
	    STATE_X = 0,
	    STATE_Y = 1,
	    STATE_THETA = 2,
	    STATE_NR_DIMS = 3
	};

    void SetupFromFile(const char fname[]);

    double GetRobotCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetRobotCenterY(void) const
    {
	return m_circles[1];	
    }

    double GetRobotRadius(void) const
    {
    	return m_circles[2];
    }
    
    const State& GetRobotState(void) const
    {
    	return m_state;
    }

    double GetGoalCenterX(void) const
    {
	return m_circles[3];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[4];	
    }

    const State GetGoalState(void) const
	{
    	auto s = State(this->GetGoalCenterX(), this->GetGoalCenterY(), 0.0);

    	return s;
	}

    double GetGoalRadius(void) const
    {
	return m_circles[5];
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 2;
    }
    
    double GetObstacleCenterX(const int i) const
    {
	return m_circles[6 + 3 * i];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[7 + 3 * i];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[8 + 3 * i];
    }
    
    double GetDistanceFromRobotCenterToGoal(void) const
    {
	const double rx = GetRobotCenterX();	
	const double ry = GetRobotCenterY();	
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	
	return sqrt((rx - gx) * (rx - gx) + (ry - gy) * (ry - gy));
    }

    bool HasRobotReachedGoal(void) const
    {
	return GetDistanceFromRobotCenterToGoal() <= GetGoalRadius();
    }

    void SetRobotState(const double s[])
    {
	SetRobotCenter(s[0], s[1]);
    }
    
    void SetRobotState(const State& s)
	{
    	SetRobotCenter(s.x, s.y);
    	m_state = s;
	}

    void SetRobotCenter(const double x, const double y)
    {
	m_circles[0] = x;
	m_circles[1] = y;
    }
    
    bool IsValidState(void) const;
  
    double GetDistOneStep(void) const
    {
    	return m_distOneStep;
    }

    double GetTimeOneStep(void) const
    {
    	return m_timeOneStep;
    }

    State SampleState() const
    {
    	auto s = State();
    	s.x = PseudoRandomUniformReal(m_bbox[0], m_bbox[2]);
    	s.y = PseudoRandomUniformReal(m_bbox[1], m_bbox[3]);
    	s.theta = PseudoRandomUniformReal()*2*acos(1);

    	return s;
    }
    

    const double* GetBoundingBox(void) const
    {
	return m_bbox;
    }

protected:    
    std::vector<double> m_circles;
    double              m_distOneStep;
    double 				m_timeOneStep;
    double              m_bbox[4];
    State			m_state;

    friend class Graphics;
};

#endif
