#include "tic_toc.h"
#include <stdio.h>

#ifdef WIN32

LARGE_INTEGER start, stop, quantum;
double time_all;

LARGE_INTEGER start_1, stop_1;
double time_all_1;

void tic_init()
{
	QueryPerformanceFrequency(&quantum);
}

void tic()
{
	QueryPerformanceCounter(&start);
}

double toc(const char* func_name)
{
	QueryPerformanceCounter(&stop);
	time_all = (double)(stop.QuadPart - start.QuadPart)/(double)(quantum.QuadPart)*1000;
	//printf("The function %s took: %f ms\n", func_name, time_all);

	return time_all;
}

LARGE_INTEGER tic_1()
{
	QueryPerformanceCounter(&start_1);

	return start_1;
}

double toc_1(LARGE_INTEGER start_mt)
{
	QueryPerformanceCounter(&stop_1);
	time_all_1 = (double)(stop_1.QuadPart - start_mt.QuadPart)/(double)(quantum.QuadPart)*1000;
	//printf("The function %s took: %f ms\n", func_name, time_all);

	return time_all_1;
}

double getTimeStamp()
{
	QueryPerformanceCounter(&stop_1);
	time_all_1 = (double)(stop_1.QuadPart)/(double)(quantum.QuadPart)*1000;

	return time_all_1;
}

#endif

#ifdef c6748

uint64_t g_start_time, g_end_time, g_overhead;
uint64_t g_total_time = 0 ;

#endif
