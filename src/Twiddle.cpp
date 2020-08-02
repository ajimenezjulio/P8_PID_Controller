#include <iostream>
#include <climits>
#include <vector>
#include "PID.h"
#include "Twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(std::vector<double> p_, std::vector<double> dp_) {
	// Parameters to tune and increments
	p = p_;
	dp = dp_;
	// Current parameter index being trated and best_error
	index = 0;
	best_err = (double) INT_MAX;
	// Flags to activate different cases in the algorithm
	isImproving = true;
	// Keep track of the number of iterations
	iteration = 0;
}

void Twiddle::PrintResults() {
	std::cout << "P = [ " << p[0] <<","<< p[1] << "," << p[2] << "]" << std::endl;
	std::cout << "DP = [ " << dp[0] <<","<< dp[1] << "," << dp[2] << "]" << std::endl;
	std::cout << "Best Error " << best_err << std::endl;
}

PID Twiddle::UpdateTwiddle(PID pid, double err) {
	// Increase counter
	iteration++;

	// If improving
	if(err < best_err) {
		isImproving = true;
		return Twiddle::DidImprove(pid, err);
	} 
	// Try on the other direction
	else if (isImproving) {
		p[index] -= 2 * dp[index];
		pid.Init(p[0], p[1], p[2]);
		isImproving = false;
	}
	// If no improvement
	else {
		isImproving = true;
		return Twiddle::DidNotImprove(pid, err);
	}

	return pid;
}

PID Twiddle::DidImprove(PID pid ,double err) {
	// Update error
	best_err = err;
	// Update increment
	dp[index] *= 1.1;
	// Prepare next index for the next iteration
	index = (index + 1) % 3;
	p[index] += dp[index];
	
  	pid.Init(p[0], p[1], p[2]);
  	return pid;
}

PID Twiddle::DidNotImprove(PID pid ,double err) {
	// Return value to original state
	p[index]+= dp[index];
	// Update increment
	dp[index] *= 0.9;
	// Prepare next index for the next iteration
	index = (index + 1) % 3;
	p[index]+= dp[index];
	
  	pid.Init(p[0], p[1], p[2]);
	return pid;
}

double Twiddle::GetBestError() {
	return best_err;
}