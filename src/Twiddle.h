#ifndef TWIDDLE_H
#define TWIDDLE_H

class Twiddle {
 public:
  /**
   * Current iteration
   */
  int iteration;
  
  /**
   * Constructor
   *
   */
  Twiddle();
  
  /**
   * Constructor
   * @param (p, dp) The initial values and the increments
   */
  void Init(std::vector<double> p_, std::vector<double> dp_);

  /**
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Print status of twiddle
   */
  void PrintResults();

  /**
   * Update twiddle values
   * @param (pid, err) pid object to return, and current cte eror
   */
  PID UpdateTwiddle(PID pid, double err);

  /**
   * Return best error
   */
  double GetBestError();


 private:
  /**
   * Parameters to tune `p` and increments `dp`
   */ 
  std::vector<double> p;
  std::vector<double> dp;

  /**
   * Current index to check and best error
   */ 
  int index;
  double best_err;

  /**
   * Flag to trigger different logic in the algorithm
   */ 
  bool isImproving;

  /**
   * Improving logic for twiddle algorithm
   * @param (pid, err) pid object to return, and current cte eror
   */ 
  PID DidImprove(PID pid, double err);

  /**
   * Non-Improving logic for twiddle algorithm
   * @param (pid, err) pid object to return, and current cte eror
   */ 
  PID DidNotImprove(PID pid, double err);
};

#endif  // TWIDDLE_H