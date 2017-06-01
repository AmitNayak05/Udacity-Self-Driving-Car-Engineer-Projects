#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K1, double K2, double K3) {
	Kp = K1;
	Ki = K2;
	Kd = K3;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	dp = Kp/10;
	di = Ki/10;
	dd = Kd/10;
	twiddle_count=0;
	first_twiddle = false;
	best_twiddle_error = 0.0;
	error_twiddle_acc = 0.0;
	step_count = 0;
	done_twiddle = false;
}

void PID::UpdateError(double cte) {
	step_count++;
	double last_cte = p_error;
	p_error=cte;
	i_error+=cte;
	if (i_error > 1000) {
		i_error = 0;
	}
	d_error=cte- last_cte;
	error_twiddle_acc = error_twiddle_acc + (cte*cte);
}

double PID::TotalError() {
	double total_error = (Kp*p_error) + (Ki*i_error) + (Kd*d_error);
	return total_error;
}

void PID::Twiddle() {
	if (!first_twiddle) {
		best_twiddle_error = error_twiddle_acc/100;
		first_twiddle = true;
		Kp = Kp + dp;
	}
	else {
		twiddle_count++;
		double p[3] = { Kp,Ki,Kd };
		double ddd[3] = { dp,di,dd };

		switch (twiddle_count)
		{
		case 1:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				dp = dp*1.1;
			}
			else {
				Kp = Kp - (2 * dp);
			}
			break;
		case 2:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				dp = dp*1.1;
			}
			else {
				Kp = Kp + dp;
				dp = dp*0.9;
			}
			Ki = Ki + di;
			break;
		case 3:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				di = di*1.1;
			}
			else {
				Ki = Ki - (2 * di);
			}
			break;
		case 4:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				di = di*1.1;
			}
			else {
				Ki = Ki + di;
				di = di*0.9;
			}
			Kd = Kd + dd;
			break;
		case 5:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				dd = dd*1.1;
			}
			else {
				Kd = Kd - (2 * dd);
			}
			break;
		case 6:
			if (error_twiddle_acc / 100 < best_twiddle_error) {
				best_twiddle_error = error_twiddle_acc / 100;
				dd = dd*1.1;
			}
			else {
				Kd = Kd + dd;
				dd = dd*0.9;
			}
			if (dp + di + dd < 0.1) {
				done_twiddle = true;
			}
			else {
				Kp = Kp + dp;
			}
			twiddle_count = 0;
			break;
		default:
			break;
		}
	
	}
	

		
	
	
	error_twiddle_acc = 0.0;
}


