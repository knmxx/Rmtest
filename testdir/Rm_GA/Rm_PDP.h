#pragma once
#include <Windows.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <math.h>
#include "Rm_job.h"
#include "Rm_ListAlgorithm.h"
#include "Rm_single_dp.h"
using namespace std;

const int PDP_INF = 1000007;

int assigned_machine[MAX_N];    //use to record the machine to process each job
int best_case_cost = PDP_INF;

Job machine[MAX_M][MAX_N] = { 0,0 };
int job_num[MAX_M] = { 0 };
Job pdpjob_indices[MAX_N];
boolean pdp_time_out = false;
double pdp_running_time;
LARGE_INTEGER cpu_freq, t1, t2, time_pdp;



int estimate()
{
	int sum_proc[MAX_M] = { 0 }, max_due[MAX_M] = { 0 };//sum_proc[MAX_M]：每台机器上的总加工时间
	int machine_id = -1, max_due_date_process_time = 0, late_work = 0, lb = 0, total_late = 0;

	for (int i = 0; i < MAX_M; i++)
		job_num[i] = 0;//int job_num[MAX_M] = { 0 };：每台机器上有多少job

	for (int i = 0; i < current_config.n; i++)
	{
		machine_id = (assigned_machine[i]) % current_config.m;//该工件分配到的机器id
		machine[machine_id][job_num[machine_id]++] = pdpjob_indices[i];//Job machine[MAX_M][MAX_N];
		//cout << machine[machine_id][job_num[machine_id] - 1].proc_time << "," << machine[machine_id][job_num[machine_id] - 1].due_date << endl;
		sum_proc[machine_id] += pdpjob_indices[i].proc_time[machine_id];
		max_due[machine_id] = max(max_due[machine_id], pdpjob_indices[i].due_date);
	}
	//带下界
	for (int i = 0; i < current_config.m; i++)
		lb += max(0, sum_proc[i] - max_due[i]);
	if (lb >= best_case_cost) return PDP_INF;
	//------------------------------

	for (int i = 0; i < current_config.m; i++)
	{
		total_late += dp_p1_single_dp(i,machine[i], job_num[i]);
		if (total_late >= best_case_cost) return PDP_INF;
	}
	return total_late;
}

void solve(int current_job_index)
{
	QueryPerformanceCounter(&time_pdp);
	if ((time_pdp.QuadPart - t1.QuadPart) / cpu_freq.QuadPart > 300)
	{
		pdp_time_out = true;
		return;
	}
	if (current_job_index == current_config.n)
	{
		int cost = estimate();
		if (cost < best_case_cost)
			best_case_cost = cost;
	}
	else
	{
		for (int i = 0; i < current_config.m; i++)
		{
			assigned_machine[current_job_index] = i;
			solve(current_job_index + 1);
		}
	}
}

int pdp_solve()
{
	
	best_case_cost = PDP_INF;
	memcpy(pdpjob_indices, job_set, sizeof(job_set));
	sort(pdpjob_indices, pdpjob_indices + current_config.n, cmp_edd);
	memset(assigned_machine, 0, sizeof(int) * MAX_N);
	QueryPerformanceFrequency(&cpu_freq);
	QueryPerformanceCounter(&t1);

	solve(0);

	QueryPerformanceCounter(&t2);

	pdp_running_time = 1.0 * (t2.QuadPart - t1.QuadPart) / cpu_freq.QuadPart;
	if (pdp_time_out == true)
		return RUN_TIME_OUT;
	return best_case_cost;
}



