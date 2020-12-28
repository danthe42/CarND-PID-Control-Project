#include <assert.h>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Set_Train_SampleLen(int _total_samplelen) {
	total_samplelen = _total_samplelen;
}

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
	samplenum = 0;
	total_cte_err = 0;
	total_cte_len = 0;
	sum_spd = 0;
}

void PID::UpdateError(double cte, double speed, double angle) {
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

#ifdef USE_SPEED_WEIGHT
	spd_error = 2000*exp( -speed/50.0 );
#else 
	spd_error = 0;
#endif

#ifdef USE_ANGLE_WEIGHT
	angle_error = 1000 * ( 1 - exp(-abs(angle) / 25.0) );
#else 
	angle_error = 0;
#endif

//	if (total_samplelen < samplenum * 2)
	{
		total_cte_err += cte * cte + spd_error + angle_error;
		sum_spd += speed;
		total_cte_len++;
	}
}

double PID::GetTotalErrorOnWholeSample(PIDTRAINER * pt) {

#ifdef USE_LOGGING
	if (pt)
	{
		pt->logfile << "AVG speed was: " << (sum_spd / total_cte_len) << " mph." << endl;
	}
#endif 
	return total_cte_err / total_cte_len;
}

double PID::TotalError() {
  samplenum++;
  return -p_error -i_error -d_error;  
}

// PIDTRAINER

PIDTRAINER::PIDTRAINER(PID* _pid, int _target_samplenum, double _p, double _i, double _d, double d1, double d2, double d3)
{
	target_samplenum = _target_samplenum;
	pid = _pid;
	deltas[0] = d1;
	deltas[1] = d2;
	deltas[2] = d3;
	params[0] = _p;
	params[1] = _i;
	params[2] = _d;
	curparamidx = 0;
	curstate = START;
	pid->Set_Train_SampleLen(target_samplenum);
#ifdef USE_LOGGING
	logfile.open("log.txt");
#endif 
}

PIDTRAINER::~PIDTRAINER()
{
#ifdef USE_LOGGING
	logfile << "---END---" << endl;
	logfile.close();
#endif
}

void PIDTRAINER::ready()
{
	if (curstate != START)
	{
#ifdef USE_LOGGING
		logfile << "Best err: " << best_err << " Params: " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " " << best_deltas[0] << " " << best_deltas[1] << " " << best_deltas[2] << " Cur_deltas[" << deltas[0] << " " << deltas[1] << " " << deltas[2] << "]" << endl;
		logfile.flush();
		prev_best = best_err;
#endif
	}

	double err;

	auto execstart = [this] {
		params[curparamidx] += deltas[curparamidx];
		pid->Init(params[0], params[1], params[2]);
		curstate = PARAMINCREASED;
	};

	switch (curstate) {
	case START:
		thisisbest(pid->GetTotalErrorOnWholeSample(this));

#ifdef USE_LOGGING
		logfile << "START Best err: " << best_err << " Params: " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " " << best_deltas[0] << " " << best_deltas[1] << " " << best_deltas[2] << endl;
		logfile.flush();
#endif

		// while 
		curparamidx = 0;
		execstart();
		break;
	case PARAMINCREASED:
		err = pid->GetTotalErrorOnWholeSample(this);
#ifdef USE_LOGGING
		logfile << "state=" << int(curstate) << " curparamidx=" << curparamidx << " cur_err=" << err << endl;
		logfile.flush();
#endif

		if (err < best_err) {
			thisisbest(err);
			deltas[curparamidx] *= 1.1;
			curparamidx = (++curparamidx) % 3;
			execstart();
		}
		else
		{
			params[curparamidx] -= 2 * deltas[curparamidx];
			pid->Init(params[0], params[1], params[2]);
			curstate = PARAMDECREASED;
		}
		break;
	case PARAMDECREASED:
		err = pid->GetTotalErrorOnWholeSample(this);
#ifdef USE_LOGGING
		logfile << "state=" << int(curstate) << " curparamidx=" << curparamidx << " cur_err=" << err << endl;
		logfile.flush();
#endif

		if (err < best_err)
		{
			thisisbest(err);

			deltas[curparamidx] *= 1.1;
			curparamidx = (++curparamidx) % 3;
			execstart();
		}
		else
		{
			params[curparamidx] += deltas[curparamidx];
			deltas[curparamidx] *= 0.9;
			curparamidx = (++curparamidx) % 3;
			execstart();
		}
		break;
	default:
		assert(false);
	}
};

void PIDTRAINER::thisisbest(double err)
{
	best_err = err;
	best_params[0] = params[0];
	best_params[1] = params[1];
	best_params[2] = params[2];
	best_deltas[0] = deltas[0];
	best_deltas[1] = deltas[1];
	best_deltas[2] = deltas[2];

#ifdef USE_LOGGING
	logfile << "NEW Best was born: " << best_err << " Params: " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " " << best_deltas[0] << " " << best_deltas[1] << " " << best_deltas[2] << endl;
	logfile.flush();
#endif

};

