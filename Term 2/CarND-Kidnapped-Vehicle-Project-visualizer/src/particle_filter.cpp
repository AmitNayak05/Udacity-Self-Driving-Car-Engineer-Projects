
/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#define _USE_MATH_DEFINES
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include<iomanip>
#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

default_random_engine gen;


double bivariate_normal(double x, double y, double mu_x, double mu_y, double sig_x, double sig_y) {
	return exp(-((x - mu_x)*(x - mu_x) / (2.0*sig_x*sig_x) + (y - mu_y)*(y - mu_y) / (2.0*sig_y*sig_y))) / (2.0*M_PI*sig_x*sig_y);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. 
	
	
	//Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 5;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);
	particles.resize(num_particles);
	weights.resize(num_particles);
	for (int i = 0; i < num_particles; i++) {
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_psi(gen);
		particles[i].weight = 1.0 / ((float)num_particles);
		weights[i] = 1.0/((float)num_particles);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::vector<Particle> particles_new;
	
	for (int i = 0; i < num_particles; i++) {
		double x, y, theta;
		
		Particle &p= particles[i];

		x = p.x;
		y = p.y;
		theta = p.theta;

		if (fabs(yaw_rate) < 0.001) { yaw_rate = 0.001; }
		//cout << "yaw rate=" << yaw_rate << endl;
		x = x + ((velocity/ yaw_rate)*(sin(theta + (yaw_rate*delta_t)) - sin(theta) ));
		y = y - ((velocity / yaw_rate)*(cos(theta + (yaw_rate*delta_t))-cos(theta)));
		theta = theta + (yaw_rate*delta_t);
		//cout << "x=" << x <<"y="<<y<<"Theta"<<theta<< endl;

		normal_distribution<double> dist_x(x, std_pos[0]);
		normal_distribution<double> dist_y(y, std_pos[1]);
		normal_distribution<double> dist_psi(theta, std_pos[2]);
		
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	for (int i = 0; i < observations.size(); i++) {
		LandmarkObs obs = observations[i];
		
		double dist_var = 10000000000;
		int index = 0;
		double distance = 0.0;
		
		for (int j = 0; j < predicted.size(); j++) {

			LandmarkObs pred = predicted[j];
			distance = dist(pred.x, pred.y, obs.x, obs.y);
			if (distance < dist_var) {
				dist_var = distance;
				index = j;
			}

		}

		observations[i].id = index;
	//	cout << "observation " << obs.x << " " << obs.y << endl;
		//cout << "Map point " << predicted[index].x << " " << predicted[index].y << endl;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	for (int i = 0; i < num_particles; i++) {
		Particle &p = particles[i];
		std::vector<LandmarkObs> map_obs;
		double theta = -p.theta;
		
		//converting observations to map coordinate frame
		for (int j = 0; j < observations.size(); j++) {
			LandmarkObs obs = observations[j];
			if (dist(obs.x, obs.y, 0, 0) <= sensor_range) {
				LandmarkObs temp_obs;
				temp_obs.x = p.x + (obs.x*cos(theta)) + (obs.y*sin(theta));
				temp_obs.y = p.y + (obs.y*cos(theta)) - (obs.x*sin(theta));
				map_obs.push_back(temp_obs);
			}
		}
		
		//converting map-landmarks to landmarks
		std::vector<LandmarkObs> map_landmark_obs;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			Map::single_landmark_s landmark = map_landmarks.landmark_list[j];
			LandmarkObs obs;
			obs.x = landmark.x_f;
			obs.id = landmark.id_i;
			obs.y = landmark.y_f;
			map_landmark_obs.push_back(obs);
		}


		dataAssociation(map_landmark_obs,map_obs);
		double w = 1.0;
		for (int j = 0; j < map_obs.size(); j++) {
			//w = w*bivariate_normal(map_landmark_obs[map_obs[j].id].x, map_landmark_obs[map_obs[j].id].y, map_obs[j].x, map_obs[j].y,std_landmark[0],std_landmark[1]);
			w*=bivariate_normal(map_obs[j].x, map_obs[j].y, map_landmark_obs[map_obs[j].id].x, map_landmark_obs[map_obs[j].id].y, std_landmark[0], std_landmark[1]);
		}

		p.weight = w;

	}


	double weight_sum = 0.0;
	for (int k = 0; k < num_particles; k++) {
		weight_sum += particles[k].weight;
	}
	for (int k = 0; k < num_particles; k++) {
		particles[k].weight = particles[k].weight / weight_sum;
		weights[k] = particles[k].weight;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::discrete_distribution<> w(weights.begin(), weights.end());
	std::vector<Particle> new_particles;

	for (int i = 0; i < num_particles; i++) {
		auto index = w(gen);
		new_particles.push_back(particles[index]);
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
