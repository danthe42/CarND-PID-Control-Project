#ifndef PID_H
#define PID_H
#include <iostream>
#include <fstream>

// If you want to use logging for PIDTRAINER, enable this
#define USE_LOGGING

// If you want to use PIDTRAINER, enable this
#define USE_TRAINING

// To use Speed_weight with in the PIDTRAINER full sample error, uncomment this
//#define USE_SPEED_WEIGHT

// To use Steering angle weight in the PIDTRAINER full sample error, uncomment this
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

  /**
   * Calculate the cost value on the whole simulation executed previously, with speed/steering_angle error(s) included if used.
   * @param pt The PIDTRAINER. Only necessary because of the logfile's handle in it.
   * @output The total cost value of this simulation run. This is the average of ( the sum of the squares of all track deviations (CTE*CTE) and speed/steering_angle error(s) if included ).
   */
  double GetCostValue(class PIDTRAINER* pt = nullptr);

  /**
   * Set length of one simulation run in training mode (with PIDTRAINER).
   * @param _total_samplelen The length of one simulation run. (The UpdateError should be called this many times) It is only used if for example the second half of a simulation is used for cost value calculation.
   */
  void Set_Train_SampleLen(int _total_samplelen);

  int samplenum;				// the number of the current iteration (starts from 0, increased on each PID controller usage)

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

  bool bFirstUpdate;			// At the first update, there's no previous CTE
  double prev_cte;				// previous Cross-track error. Used for D error calculation
  double sum_cte;				// Sum of CTEs. Used for I error calculation
  double sum_spd;				// Sum of car's current speed values. Only used for logging the average speed of the car in the simulation runs
  double total_cte_err;			// the accumulated error values used for cost value
  int total_cte_len;			// The count of the items summarized in total_cte_err.  ( at the end, total_cte_err/total_cte_err will be the average cost value )
  int total_samplelen;			// The length of one simulation run (UpdateError will be called this many times)
};

// PIDTRAINER class: 
//   This class implements a PID hyperparameter trainer.
//   It requires an initial P,I,and D parameter, and 3 delta values to be used in the twiddle algorithm. (@see notes.md)
//	 It can be used by calling its ready() method after a simulation run is over. 
class PIDTRAINER {

public:
	ofstream logfile;
	PID* pid;					// the PID controller to train

	// the current parameters and deltas
	double params[3];
	double deltas[3];

	// the lowest cost value, parameters and deltas
	double best_err;
	double best_deltas[3];
	double best_params[3];
	
	// simulation run length
	int target_samplenum;

	// States used in the twiddle algorithm
	enum {
		START,
		PARAMINCREASED,
		PARAMDECREASED,
	} curstate;
	
	// The index of the parameter to be used currently in the twiddle algorithm
	int curparamidx;

	/**
	* Construct the PID trainer
	* @param _pid The PID controller to train
	* @param _target_samplenum The length of one simulation run
	* @param _p,_i,_d,d1,d2,d3 The initial PID coefficients with delta values used in the twiddle algorithm
	*/	
	PIDTRAINER(PID* _pid, int _target_samplenum, double _p, double _i, double _d, double d1, double d2, double d3);
	
	~PIDTRAINER();

	// This method implements the asynchronous twiddle algorithm
	// It must be called every time when a simulation run is finished
	void ready();

private:
	// This internal method will be called when new best hyperparameters are found by the algorithm (called from the ready() method)
	void best_found(double err);
};

#endif  // PID_H