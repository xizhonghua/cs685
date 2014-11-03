/*
 * MySVM.cpp
 *
 *  Created on: Nov 2, 2014
 *      Author: zhonghua
 */

#include "MySVM.h"
#include "MP.hpp"

const string model_file_name = "dw.model";

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

MySVM::MySVM() {
	this->x_space = NULL;
	this->model = NULL;

	this->Init();
}

MySVM::~MySVM() {
	svm_free_and_destroy_model(&model);
	svm_destroy_param(&param);
	free(prob.y);
	free(prob.x);
	free(x_space);
}


void MySVM::Init()
{
	// default values
	param.svm_type = NU_SVR;
	param.kernel_type = RBF;
	param.degree = 3;
	param.gamma = 0;	// 1/num_features
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 100;
	param.C = 1;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 0;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;
}

void MySVM::LoadModel()
{
	model=svm_load_model(model_file_name.c_str());
	if(!model)
	{
		fprintf(stderr,"can't open model file %s\n", model_file_name.c_str());
		exit(1);
	}
	else
	{
		cerr<<" - svm loaded from "<<model_file_name<<endl;
	}

}

void MySVM::Train(const vector<Example>& examples)
{
	const int NumFeatures = 3;	// dx, dy, dtheta

	this->prob.l = examples.size();
	this->prob.x = Malloc(struct svm_node *,prob.l);
	this->prob.y = Malloc(double, prob.l);
	this->x_space = Malloc(struct svm_node, prob.l * (NumFeatures+1));

	for(auto i=0;i<this->prob.l;++i)
	{
		this->prob.x[i] = &x_space[i*3];
		this->prob.y[i] = examples[i].time;

		x_space[i*4].index = 0;
		x_space[i*4+1].index = 1;
		x_space[i*4+2].index = 2;
		x_space[i*4+3].index = -1;		// end

		x_space[i*4].value = examples[i].dx;
		x_space[i*4+1].value = examples[i].dy;
		x_space[i*4+2].value = examples[i].dtheta;
	}

	this->param.gamma = 1.0 / NumFeatures;


	do_cross_validation();

	model = svm_train(&prob,&param);
	svm_save_model(model_file_name.c_str(),model);
}



double MySVM::Predict(const State& st1, const State& st2)
{
	svm_node x[4];

	x[0].index = 0;
	x[1].index = 1;
	x[2].index = 2;
	x[3].index = -1;

	x[0].value = st2.x - st1.x;
	x[1].value = st2.y - st1.y;
	x[2].value = MotionPlanner::AngleDiff(st1.theta, st2.theta);

	auto dist = svm_predict(model, x);

	// cerr<<"dist = "<<dist<<" se2 = "<<MotionPlanner::DistSE2(st1,st2)<<endl;
	return dist;
}


void MySVM::do_cross_validation()
{
	int i;
	int total_correct = 0;
	double total_error = 0;
	double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
	double *target = Malloc(double,prob.l);

	int nr_fold = 3;

	svm_cross_validation(&prob,&param,nr_fold,target);
	if(param.svm_type == EPSILON_SVR ||
	   param.svm_type == NU_SVR)
	{
		for(i=0;i<prob.l;i++)
		{
			double y = prob.y[i];
			double v = target[i];
			total_error += (v-y)*(v-y);
			sumv += v;
			sumy += y;
			sumvv += v*v;
			sumyy += y*y;
			sumvy += v*y;
		}
		printf("Cross Validation Mean squared error = %g\n",total_error/prob.l);
		printf("Cross Validation Squared correlation coefficient = %g\n",
			((prob.l*sumvy-sumv*sumy)*(prob.l*sumvy-sumv*sumy))/
			((prob.l*sumvv-sumv*sumv)*(prob.l*sumyy-sumy*sumy))
			);
	}
	else
	{
		for(i=0;i<prob.l;i++)
			if(target[i] == prob.y[i])
				++total_correct;
		printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/prob.l);
	}
	free(target);
}
