/*
*	File: simulation_elements.c
*	
*	Implements various simulation elements	
*
*	Author: Tobias Braechter
*	Date: 16.11.2017
*
*
*/

#include <stdlib.h>
#include "simulation_elements.h"

void compute_link(sim_object *this, double time)
{
	if(this)
	{
		if(this->base.enabled)
		{
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				switch(this->link.link)
				{
					case ADD:
						*(this->link.out) = (*(this->link.in1)) + (*(this->link.in2));
						break;
					case SUBSTRACT:
						*(this->link.out) = (*(this->link.in1)) - (*(this->link.in2));
						break;
					case MULTIPLY:
						*(this->link.out) = (*(this->link.in1)) * (*(this->link.in2));
						break;
					case DIVIDE:
						*(this->link.out) = (*(this->link.in1)) / (*(this->link.in2));
						break;
				}
				this->base.lastCompute = time;
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void compute_pt1(sim_object *this, double time)
{
	
	if(this)
	{
		if(this->base.enabled)
		{
			if(this->base.reset)
			{
				this->pt1.outSum = 0;
				this->base.reset = 0;
			}
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				double diff = (this->pt1.k * *(this->base.in1) - this->pt1.outSum)/this->pt1.t1;
				this->pt1.outSum = this->pt1.outSum + diff * timeDiff;
				*(this->base.out) = this->pt1.outSum;
				this->base.lastCompute = time;
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void compute_pid(sim_object *this, double time)
{
	if(this)
	{
		if(this->base.enabled)
		{
			if(this->base.reset)
			{
				this->pid.eSum = 0;
				this->pid.eLast = 0;
				this->base.reset = 0;
			}
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				if(this->pid.eLast > 0 && this->pid.eLast - *(this->pid.in1) > this->pid.eLast)
					this->pid.eSum = 0;
				else if(*(this->pid.in1) > 0 && *(this->pid.in1) - this->pid.eLast > *(this->pid.in1))
					this->pid.eSum = 0;
				else
					this->pid.eSum = this->pid.eSum + *(this->pid.in1) * timeDiff;
				if(this->pid.eSum > this->pid.maxEsum)
					this->pid.eSum = this->pid.maxEsum;
				if(this->pid.eSum < -(this->pid.maxEsum))
					this->pid.eSum = -(this->pid.maxEsum);
				double diff = (*(this->pid.in1) - this->pid.eLast) / timeDiff;
				double out = this->pid.kp * *(this->pid.in1);
				out += this->pid.ki * this->pid.eSum;
				out += this->pid.kd * diff;
				out += this->pid.offset;
				if(out > this->pid.max)
					out = this->pid.max;
				if(out < this->pid.min)
					out = this->pid.min;
				this->pid.eLast = *(this->pid.in1);
				this->base.lastCompute = time;
				*(this->pid.out) = out;			
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void compute_integrator(sim_object *this, double time)
{
	
	if(this)
	{
		if(this->base.enabled)
		{
			if(this->base.reset)
			{
				this->integrator.eSum = 0;
				this->base.reset = 0;
			}
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				this->integrator.eSum = this->integrator.eSum + *(this->base.in1) * timeDiff;
				*(this->base.out) = this->integrator.eSum;
				this->base.lastCompute = time;
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void compute_dif(sim_object *this, double time)
{
	
	if(this)
	{
		if(this->base.enabled)
		{
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				*(this->base.out) = (*(this->dif.in1) - this->dif.eLast) / timeDiff;
				this->dif.eLast = *(this->dif.in1);
				this->base.lastCompute = time;
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void compute_lim(sim_object *this, double time)
{
	if(this)
	{
		if(this->base.enabled)
		{
			double timeDiff = time - this->base.lastCompute;
			if(timeDiff >= this->base.sampleTime)
			{
				double out = *(this->base.in1);
				if(out < this->lim.min)
					out = this->lim.min;
				if(out > this->lim.max)
					out = this->lim.max;
				*(this->base.out) = out;
			}
		}
		else
			*(this->base.out) = 0;
	}
}

void setInterfaces(sim_object *this, double *in1, double *in2, double *out)
{
	if(this)
	{
		this->base.in1 = in1;
		this->base.in2 = in2;
		this->base.out = out;
	}
}

void setSampleTime(sim_object *this, double sampleTime)
{
	if(this)
	{
		this->base.sampleTime = sampleTime;
	}
}

void linkSetParams(sim_object *this, uint8_t link)
{
	if(this)
	{
		this->link.link = link;
	}
}

void pt1SetParams(sim_object *this, double t1, double k)
{
	if(this)
	{
		this->pt1.t1 = t1;
		this->pt1.k = k;
	}
}

void pidSetParams(sim_object *this, double kp, double ki, double kd)
{
	if(this)
	{
		this->pid.kp = kp;
		this->pid.ki = ki;
		this->pid.kd = kd;
	}
}

void pidSetLimits(sim_object *this, double min, double max, double maxEsum)
{
	if(this)
	{
		this->pid.min = min;
		this->pid.max = max;
		this->pid.maxEsum = maxEsum;
	}
}

void pidSetOffset(sim_object *this, double offset)
{
	if(this)
	{
		this->pid.offset = offset;
	}
}

void limSetParams(sim_object *this, double min, double max)
{
	if(this)
	{
		this->lim.min = min;
		this->lim.max = max;
	}
}


void reset(sim_object *this, double time)
{
	if(this)
	{
		this->base.lastCompute = time;
		this->base.reset = 1;
		*(this->base.out) = 0;
	}
}

void setEnabled(sim_object *this, uint8_t enabled)
{
	if(this)
	{
		this->base.enabled = enabled;
	}
}

double getOutput(sim_object *this)
{
	if(this)
	{
		return *(this->base.out);
	}
	return 0;
}

void basicSettings(sim_object *this, double sampleTime)
{
	if(this)
	{
		this->base.enabled = 1;
		this->base.reset = 0;
		this->base.sampleTime = sampleTime;
		this->base.lastCompute = 0;
	}
}

sim_object *createLink(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_link));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_link;
	return object;
}

sim_object *createPT1(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_pt1));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_pt1;
	return object;
}

sim_object *createPID(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_pid));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_pid;
	object->pid.eSum = 0;
	object->pid.eLast = 0;
	pidSetLimits(object, 0, 0, 0);
	pidSetOffset(object, 0);
	return object;
}

sim_object *createIntegrator(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_integrator));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_integrator;
	object->integrator.eSum = 0;
	return object;
}

sim_object *createDIF(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_dif));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_dif;
	object->dif.eLast = 0;
	return object;
}

sim_object *createLIM(double sampleTime)
{
	sim_object *object = malloc(sizeof(sim_lim));
	basicSettings(object, sampleTime);	
	object->base.compute = compute_lim;
	object->lim.min = 0;
	object->lim.max = 0;
	return object;
}

void deleteSimObject(sim_object *this)
{
	if(this != 0)
		free(this);
	//überarbeiten!!!!!!
}
