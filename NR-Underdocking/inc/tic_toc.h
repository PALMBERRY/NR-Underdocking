#ifndef _TIC_TOC_H
#define _TIC_TOC_H

#ifdef WIN32

#include <Windows.h>

#ifdef __cplusplus
extern "C" {
#endif

void tic_init();

void tic();

double toc(const char* func_name);

LARGE_INTEGER tic_1();

double toc_1(LARGE_INTEGER start_mt);

double getTimeStamp();

#ifdef __cplusplus
}
#endif

#endif

#ifdef c6748

#include <c6x.h>	   // defines _itoll, TSCH, TSCL
#include <stdio.h>

// In the variable declearation portion of the code:
extern uint64_t g_start_time, g_end_time, g_overhead;
extern uint64_t g_total_time;

inline void tic_init()
{
	TSCL = 0;		//enable TSC
	g_start_time = _itoll(TSCH, TSCL);
	g_end_time = _itoll(TSCH, TSCL);
	g_overhead = g_end_time-g_start_time; //Calculating the overhead of the method.
}


inline void tic()
{
	g_start_time = _itoll(TSCH, TSCL);
}

inline double toc(const char* func_name)
{
	double use_time;
	g_end_time = _itoll(TSCH, TSCL);
	use_time = (g_end_time-g_start_time-g_overhead)*_rcpdp(456000);
	//g_total_time += g_end_time-g_start_time-g_overhead;
	//printf("The function %s took: %f ms\n", func_name, use_time);
	return use_time;
}

inline uint64_t tic_1()
{
	return _itoll(TSCH, TSCL);
}

inline double toc_1(uint64_t start_time)
{
	double use_time;
	uint64_t end_time;
	end_time = _itoll(TSCH, TSCL);
	use_time = (end_time-start_time-g_overhead)*_rcpdp(456000);
	//g_total_time += g_end_time-g_start_time-g_overhead;
	//printf("The function %s took: %f ms\n", func_name, use_time);
	return use_time;
}

inline double toc_2(uint64_t start_time, uint64_t end_time)
{
	double use_time;
	if(end_time > start_time)
		use_time = (end_time-start_time-g_overhead)*_rcpdp(456000);
	else
		use_time = -((double)(start_time-end_time-g_overhead)*_rcpdp(456000));
	//g_total_time += g_end_time-g_start_time-g_overhead;
	//printf("The function %s took: %f ms\n", func_name, use_time);
	return use_time;
}

inline double getTimeStamp()
{
	double use_time;
	uint64_t end_time;

	end_time = _itoll(TSCH, TSCL);
	use_time = (end_time)*_rcpdp(456000);

	return use_time;
}

#endif

#endif
