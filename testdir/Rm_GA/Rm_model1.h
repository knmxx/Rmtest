#pragma once
#include<gurobi_c++.h> 
#include <Windows.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <math.h>
#include "Rm_job.h"
using namespace std;

const int model1_M = 1000007;
int model1_result = -1;
Job model1_jobs[MAX_N];
double model1_running_time;



void model1_optimation()
{
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	//Step 1:Add variables
	//Step 1.1: add variables xxx_ijk
	GRBVar ***xxx;
	//xxx[0] invalid
	//xxx[1]~xxx[m]
	xxx = new GRBVar**[current_config.m + 1];
	for (int i = 1; i < current_config.m + 1; ++i)
	{
		//xxx[i][0] ~ xxx[i][n]
		xxx[i] = new GRBVar*[current_config.n + 1];
		for (int j = 0; j < current_config.n + 1; ++j)
		{
			//xxx[i][j][0] invalid
			//xxx[i][j][1]~xxx[i][j][n]
			xxx[i][j] = new GRBVar[current_config.n + 1];
		}
	}
	for (int i = 1; i < current_config.m + 1; ++i)
	{
		for (int j = 0; j < current_config.n + 1; ++j)
		{
			for (int k = 1; k < current_config.n + 1; ++k)
			{
				if (j != k)
					xxx[i][j][k] = model.addVar(0, 1, 0, GRB_BINARY);
			}
		}
	}

	//step 1.2 add variables c_j, p_j
	//j = 0, 1, ..., n
	GRBVar *p = new GRBVar[current_config.n + 1];
	GRBVar *c = new GRBVar[current_config.n + 1];
	int max_c = 0;
	//upper bound for c
	for (int i = 0; i < current_config.n; ++i)
	{
		int tmp = 0;
		for (int j = 0; j < current_config.m; ++j)
		{
			if (model1_jobs[i].proc_time[j] > tmp)
				tmp = model1_jobs[i].proc_time[j];
		}
		max_c += tmp;
	}
	p[0] = model.addVar(0, 0, 0, GRB_INTEGER);
	c[0] = model.addVar(0, 0, 0, GRB_INTEGER);
	for (int i = 1; i < current_config.n + 1; ++i)
	{
		//upper bound for p
		int tmp = 0;
		for (int j = 0; j < current_config.m; ++j)
		{
			if (model1_jobs[i-1].proc_time[j] > tmp)
				tmp = model1_jobs[i - 1].proc_time[j];
		}
		p[i] = model.addVar(0, tmp, 0, GRB_INTEGER);
		c[i] = model.addVar(1, max_c, 0, GRB_INTEGER);
	}

	//step 1.3 add auxiliary variables u_j1, u_j2, v_j1, v_j2, z_j, y_j
	GRBVar **u = new GRBVar*[current_config.n + 1];
	GRBVar **v = new GRBVar*[current_config.n + 1];
	GRBVar *z = new GRBVar[current_config.n + 1];
	GRBVar *y = new GRBVar[current_config.n + 1];
	for (int i = 1; i < current_config.n + 1; ++i)
	{
		u[i] = new GRBVar[2];
		v[i] = new GRBVar[2];
		for (int j = 0; j < 2; ++j)
		{
			u[i][j] = model.addVar(0, 1, 0, GRB_BINARY);
			v[i][j] = model.addVar(0, 1, 0, GRB_BINARY);
		}
		/*z[i] = model.addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
		y[i] = model.addVar(0, GRB_INFINITY, 0, GRB_INTEGER);*/
		z[i] = model.addVar(0, max_c, 0, GRB_INTEGER);
		y[i] = model.addVar(0, max_c, 0, GRB_INTEGER);
	}


	//Step 2: Add constraints
	for (int k = 1; k < current_config.n + 1; ++k)
	{
		GRBLinExpr cons = 0.0;
		for (int i = 1; i < current_config.m + 1; ++i)
		{
			for (int j = 0; j < current_config.n + 1; ++j)
			{
				if (j != k)
				{
					cons += xxx[i][j][k];
				}
			}
		}
		model.addConstr(cons == 1);
	}

	for (int j = 1; j < current_config.n + 1; ++j)
	{
		GRBLinExpr cons = 0.0;
		for (int i = 1; i < current_config.m + 1; ++i)
		{
			for (int k = 1; k < current_config.n + 1; ++k)
			{
				if (j != k)
				{
					cons += xxx[i][j][k];
				}
			}
		}
		model.addConstr(cons <= 1);
	}

	for (int i = 1; i < current_config.m + 1; ++i)
	{
		GRBLinExpr cons = 0.0;
		for (int k = 1; k < current_config.n + 1; ++k)
		{
			cons += xxx[i][0][k];
		}
		model.addConstr(cons <= 1);
	}

	for (int i = 1; i < current_config.m + 1; ++i)
	{
		for (int j = 1; j < current_config.n + 1; ++j)
		{
			for (int k = 1; k < current_config.n + 1; ++k)
			{
				if (j != k)
				{
					GRBLinExpr cons = 0.0;
					for (int h = 0; h < current_config.n + 1; ++h)
					{
						if (h != k && h != j)
						{
							cons += xxx[i][h][j];
						}
					}
					model.addConstr(cons >= xxx[i][j][k]);
				}
			}
		}
	}


	for (int k = 1; k < current_config.n + 1; ++k)
	{
		GRBLinExpr tmp;
		for (int i = 1; i < current_config.m + 1; ++i)
		{
			for (int j = 0; j < current_config.n + 1; ++j)
			{
				if (j != k)
					//process time of job j in machine i --> data[j - 1][i - 1]
					tmp += xxx[i][j][k] * model1_jobs[k-1].proc_time[i-1];
			}
		}
		model.addConstr(p[k] == tmp);
	}

	for (int i = 1; i < current_config.m + 1; ++i)
	{
		for (int j = 0; j < current_config.n + 1; ++j)
		{
			for (int k = 1; k < current_config.n + 1; ++k)
			{
				if (j != k)
				{
					model.addConstr(c[k] + max_c * (1 - xxx[i][j][k]) >= c[j] + p[k]);
				}
			}
		}
	}

	//add auxiliary constraints
	for (int j = 1; j < current_config.n + 1; ++j)
	{
		model.addConstr(z[j] <= p[j]);
		model.addConstr(z[j] <= y[j]);
		model.addConstr(p[j] <= z[j] + model1_M * (1 - u[j][0]));
		model.addConstr(y[j] <= z[j] + model1_M * (1 - u[j][1]));
		/*model.addConstr(p[j] <= z[j] + max_c * (1 - u[j][0]));
		model.addConstr(y[j] <= z[j] + max_c * (1 - u[j][1]));*/
		model.addConstr(u[j][0] + u[j][1] >= 1);

		model.addConstr(y[j] >= c[j] - model1_jobs[j - 1].due_date);
		model.addConstr(0 >= y[j] - model1_M * (1 - v[j][0]));
		model.addConstr(c[j] - model1_jobs[j - 1].due_date >= y[j] - model1_M * (1 - v[j][1]));
		/*model.addConstr(0 >= y[j] - max_c * (1 - v[j][0]));
		model.addConstr(c[j] - instance.data[j - 1][instance.m] >= y[j] - max_c * (1 - v[j][1]));*/
		model.addConstr(v[j][0] + v[j][1] >= 1);
	}


	//Step 3: Set objective
	double *w = new double[current_config.n + 1];
	for (int i = 0; i < current_config.n + 1; ++i)
		w[i] = 1;
	GRBLinExpr obj = 0.0;
	for (int j = 1; j < current_config.n + 1; ++j)
	{
		obj += w[j] * z[j];
	}
	model.setObjective(obj, GRB_MINIMIZE);
	model.set(GRB_IntParam_OutputFlag, 0);
	model.optimize();

	try
	{
		//write objective value to file "output.txt"
		model1_result =  model.get(GRB_DoubleAttr_ObjVal);
	}
	catch (GRBException err)
	{
		cout << "-------------" << endl;
		cout << err.getMessage();
		cout << "-------------" << endl;
	}

}


int model1_solve()
{
	model1_result = -1;
	LARGE_INTEGER cpu_freq, t1, t2;
	memcpy(model1_jobs, job_set, sizeof(job_set));
	QueryPerformanceFrequency(&cpu_freq);
	QueryPerformanceCounter(&t1);

	model1_optimation();

	QueryPerformanceCounter(&t2);

	model1_running_time = 1.0 * (t2.QuadPart - t1.QuadPart) / cpu_freq.QuadPart;
	return model1_result;
}



