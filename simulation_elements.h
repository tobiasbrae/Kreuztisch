/*
*	File: simulation_elements.h
*	
*	Implements various simulation elements	
*
*	Author: Tobias Braechter
*	Date: 16.11.2017
*
*
*/

#include <stdint.h>

#ifndef SIMULATION_ELEMENTS_H_
#define SIMULATION_ELEMENTS_H_

	// link -> links two outputs (add, sub, mul, div...)
	// pt1 -> simulates a pt1-system
	// pid -> simulates a pid-controller
	// integrator -> simulates an integrator
	// dif -> simulates an differentiator
	// lim -> limits the given input value to the set min and max values

	#define NOTHING 0
	#define ADD 1
	#define SUBSTRACT 2
	#define MULTIPLY 3
	#define DIVIDE 4

	union sim_object;

// ===================================================[sim_base]=====================================================
	
	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
	}sim_base;
	
	
// ===================================================[sim_link]=====================================================

	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		uint8_t link;
	}sim_link;
	
// ===================================================[sim_pt1]======================================================

	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		double t1;
		double k;
		double outSum;
	}sim_pt1;
	
// ===================================================[sim_pid]=================================================
	
	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		double eSum;
		double eLast;
		double kp;
		double ki;
		double kd;
		double min;
		double max;
		double maxEsum;
		double offset;
	}sim_pid;
	
	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		double eSum;
	}sim_integrator;
	
	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		double eLast;
	}sim_dif;

// ===================================================[sim_lim]=====================================================

	typedef struct
	{
		uint8_t enabled;
		uint8_t reset;
		double *in1;
		double *in2;
		double *out;
		double sampleTime;
		double lastCompute;
		void (*compute) (union sim_object*, double);
		double min;
		double max;
	}sim_lim;
	
	
// ===================================================[constructors]=================================================
	
	typedef union sim_object
	{
		sim_base base;
		sim_link link;
		sim_pt1 pt1;
		sim_pid pid;
		sim_integrator integrator;
		sim_dif dif;
		sim_lim lim;
	}sim_object;

	void setInterfaces(sim_object *this, double *in1, double *in2, double *out);
	void setSampleTime(sim_object *this, double sampleTime);
	void linkSetParams(sim_object *this, uint8_t link);
	void pt1SetParams(sim_object *this, double t1, double k);
	void pidSetParams(sim_object *this, double kp, double ki, double kd);
	void pidSetLimits(sim_object *this, double min, double max, double maxEsum);
	void pidSetOffset(sim_object *this, double offset);
	void limSetParams(sim_object *this, double min, double max);
	void reset(sim_object *this, double time);double getOutput(sim_object *this);
	void setEnabled(sim_object *this, uint8_t enabled);
	double getOutput(sim_object *this);
	
	sim_object *createLink(double sampleTime);
	sim_object *createPT1(double sampleTime);
	sim_object *createPID(double sampleTime);
	sim_object *createIntegrator(double sampleTime);
	sim_object *createDIF(double sampleTime);
	sim_object *createLIM(double sampleTime);
	
	void deleteSimObject(sim_object *this);

#endif
