#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/LU"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }




// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event


    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  

		  vector<double> ptsx_rel;
          vector<double> ptsy_rel;

		  Eigen::VectorXd xvals(ptsx.size());
  		  Eigen::VectorXd yvals(ptsx.size());

		//convert waypoints to local vehicle coordinates

		  for(int j=0;j<ptsx.size();j++){

			double loop_x=ptsx[j]-px;
			double loop_y=ptsy[j]-py;

			double converted_x=(loop_x*cos(-psi)) - (loop_y*sin(-psi));
			double converted_y=(loop_y*cos(-psi)) + (loop_x*sin(-psi));

		  	ptsx_rel.push_back(converted_x);
			ptsy_rel.push_back(converted_y);
			xvals[j]=converted_x;
			yvals[j]=converted_y;
		  }

		  auto coeffs = polyfit(xvals, yvals, 3);

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  double cte=polyeval(coeffs, 0.0);
		  double epsi = -atan(coeffs[1]);

		  Eigen::VectorXd state(6);
		  state << 0, 0, 0, v, cte, epsi;

		  //auto mpc_output = mpc.Solve(state, coeffs);

          double steer_value;
          double throttle_value=0.3;




		  std::vector<double> x_vals = {state[0]};
		  std::vector<double> y_vals = {state[1]};
		  std::vector<double> psi_vals = {state[2]};
		  std::vector<double> v_vals = {state[3]};
		  std::vector<double> cte_vals = {state[4]};
		  std::vector<double> epsi_vals = {state[5]};
		  std::vector<double> delta_vals = {};
		  std::vector<double> a_vals = {};
		  


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

/*
int iters=10;

		  for (size_t i = 0; i < iters; i++) {
			std::cout << "Iteration " << i << std::endl;

			auto vars = mpc.Solve(state, coeffs);

			x_vals.push_back(vars[0]);
			y_vals.push_back(vars[1]);
			psi_vals.push_back(vars[2]);
			v_vals.push_back(vars[3]);
			cte_vals.push_back(vars[4]);
			epsi_vals.push_back(vars[5]);

			delta_vals.push_back(vars[6]);
			a_vals.push_back(vars[7]);

			mpc_x_vals.push_back(vars[0]);
			mpc_y_vals.push_back(vars[1]);

			state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
		}
		
*/

		auto vars = mpc.Solve(state, coeffs);

		Eigen::MatrixXd m11(3,3);
		Eigen::MatrixXd m12(3,3);
		Eigen::MatrixXd m13(3,3);

			for (size_t i = 0; i < 15; i++) {

				mpc_x_vals.push_back(vars[2*i]);
				mpc_y_vals.push_back(vars[(2*i)+1]);
	
			}

		m11<<vars[24],vars[25],1,
					vars[36],vars[37],1,
					vars[48],vars[49],1;

		m12<<(vars[24]*vars[24])+(vars[25]*vars[25]),vars[25],1,
					(vars[36]*vars[36])+(vars[37]*vars[37]),vars[37],1,
					(vars[48]*vars[48])+(vars[49]*vars[49]),vars[49],1;

		m13<<(vars[24]*vars[24])+(vars[25]*vars[25]),vars[24],1,
					(vars[36]*vars[36])+(vars[37]*vars[37]),vars[36],1,
					(vars[48]*vars[48])+(vars[49]*vars[49]),vars[48],1;

        double val_m11=m11.determinant();
		double val_m12=m12.determinant();
		double val_m13=m13.determinant();
		double radius=1000.0;
		double x0=0.0;
		double y0=0.0;
		if(val_m11!=0){
			x0=val_m12/(2*val_m11);
			y0=val_m13/(2*val_m11);

			radius=sqrt((vars[24]-x0)*(vars[24]-x0) + (vars[25]-x0)*(vars[25]-y0));
			std::cout<<"radius= "<<radius;
		}
		
		

        json msgJson;
          


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green lin    

          //Display the waypoints/reference line
          vector<double> next_x_vals;
		  vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line


 		  for(double x = 0; x <= 80; x += 5.0){
			auto v = polyeval(coeffs, x);
			double y_temp=v;
		  	next_x_vals.push_back(x);
		  	next_y_vals.push_back(y_temp);
		  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

		  msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
		  
		  msgJson["steering_angle"] = -vars[50]/0.436332;
			
		  throttle_value=vars[51];
		if(fabs(radius)<120){
			throttle_value=-0.01;
			}
		if(fabs(radius)>=120 && fabs(radius)<260){
			throttle_value=0.15;
			}
			
		
          msgJson["throttle"] =throttle_value;
			
std::cout<<"input throttke from mpc= "<<vars[51]<<endl;  

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.	
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          (*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed th
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    (*ws).close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
