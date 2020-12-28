#ifndef PID_H
#define PID_H
#include <iostream>
#include <fstream>

#define USE_LOGGING
//#define USE_TRAINING
//#define USE_SPEED_WEIGHT
//#define USE_ANGLE_WEIGHT

using std::ofstream;
using std::endl;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double speed, double angle);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  double GetTotalErrorOnWholeSample(class PIDTRAINER* pt = nullptr);
  void Set_Train_SampleLen(int _total_samplelen);

  int samplenum;
  double total_cte_err;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double spd_error;
  double angle_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  bool bFirstUpdate;
  double prev_cte;
  double sum_cte;
  double sum_spd;
  int total_cte_len;
  int total_samplelen;
};

class PIDTRAINER {

public:
	ofstream logfile;

	PID* pid;
	double deltas[3];
	double params[3];
	double best_err;
	double best_deltas[3];
	double best_params[3];
	int target_samplenum;

	enum {
		START,
		PARAMINCREASED,
		PARAMDECREASED,
	} curstate;

	int curparamidx;
	PIDTRAINER(PID* _pid, int _target_samplenum, double _p, double _i, double _d, double d1, double d2, double d3);
	~PIDTRAINER();
	double prev_best = 10000000;

	void thisisbest(double err);
	void ready();
};

#endif  // PID_H