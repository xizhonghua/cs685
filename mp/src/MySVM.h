/*
 * MySVM.h
 *
 *  Created on: Nov 2, 2014
 *      Author: zhonghua
 */

#ifndef MYSVM_H_
#define MYSVM_H_

#include <vector>
#include "MP.hPP"
#include "svm.h"
using namespace std;

struct Example
{
	double dx;
	double dy;
	double dtheta;
	double time;
};

class MySVM {
public:
	MySVM();

	void Init();

	void LoadModel();

	void Train(const vector<Example>& examples);

	double Predict(const State& st1, const State& st2);

	virtual ~MySVM();

private:

	void do_cross_validation();

	struct svm_parameter param;		// set by parse_command_line
	struct svm_problem prob;		// set by read_problem
	struct svm_model *model;
	struct svm_node *x_space;
};

#endif /* MYSVM_H_ */
