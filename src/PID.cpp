#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	prev_cte = 0;
	bFirstUpdate = true;
	sum_cte = 0;
}

void PID::UpdateError(double cte) {
	p_error = Kp * cte;
	if (bFirstUpdate)
	{
		d_error = 0;
		bFirstUpdate = false;
	}
	else
	{
		d_error = Kd * (cte - prev_cte);
	}
	prev_cte = cte;
	sum_cte += cte;
	i_error = Ki * sum_cte;
}

double PID::TotalError() {
  return -p_error -i_error -d_error;  
}