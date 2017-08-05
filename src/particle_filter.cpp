/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    //cout << "****************************  INITIALIZATION *********************" << endl;
    //cout << "x = " << x << " y= " << y << " theta= " << theta << endl;
    //cout << "******************************************************************" << endl;	
	
    num_particles = 100;
    particles.resize(num_particles);
	weights.resize(num_particles);
	
    std::default_random_engine generator;
    std::normal_distribution<double> dist_x(x,std[0]);
    std::normal_distribution<double> dist_y(y,std[1]);
	std::normal_distribution<double> dist_theta(theta,std[2]);

	for (int i=0; i < num_particles; i++){	
		// id of the particle
		particles[i].id = i;
		// x y theta position + noise
		float init_x     = dist_x(generator);
		float init_y     = dist_y(generator);
		float init_theta = dist_theta(generator);
		particles[i].x     = init_x;
		particles[i].y     = init_y;
		particles[i].theta = init_theta;			
		// set all initial weights to 1
		weights[i] = 1.0;
		particles[i].weight = 1.0;
	}
	// initialize flag
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // very important for this to be outside the particle loop to be consistent with generated values!!!!!!! 
	std::default_random_engine generator;
	
	for (int i=0; i < num_particles; i++){	
	    // prediction using constant velocity and yawrate
		particles[i].x     = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
		particles[i].y     = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;
        // noraml dist about predicted x y theta 
		std::normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		std::normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		std::normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);	
        // random value from normal dist 
		particles[i].x     = dist_x(generator);
		particles[i].y     = dist_y(generator);
		particles[i].theta = dist_theta(generator);		
		}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, 
                                   double std_landmark[], 
		                           std::vector<LandmarkObs> observations,
								   Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// for every particle
	for (int i = 0; i < particles.size(); i++){

		float prob = 1.0;

		// for every observation -> transform into global CS viewed from particle
		for (LandmarkObs obsrv : observations){
			LandmarkObs obs_tf;
			obs_tf.id = obsrv.id;
			// 2-d transformation based on yaw=theta
			obs_tf.x = particles[i].x + cos(particles[i].theta)*obsrv.x - sin(particles[i].theta)*obsrv.y;
			obs_tf.y = particles[i].y + sin(particles[i].theta)*obsrv.x + cos(particles[i].theta)*obsrv.y;		

			//cout << "*************PARTICLE=" << particles[i].id << endl;
			//cout << "x=" << particles[i].x << " y=" << particles[i].y << " theta=" << particles[i].theta << endl;	
			//cout << "x = obsrv.x= " << obsrv.x << endl;
			//cout << "y = obsrv.y= " << obsrv.y << endl;
			//cout << "x = obs_tf.x= " << obs_tf.x << endl;
			//cout << "y = obs_tf.y= " << obs_tf.y << endl;	

			// distances to each landmark (from observations)
			vector <double> distances;
			for(int j = 0; j < map_landmarks.landmark_list.size(); ++j){
				// distance from landmark to transformed observation
				double dst = dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f, obs_tf.x, obs_tf.y);
				//cout << "landm.j= " << j << " landm.x_f= " << map_landmarks.landmark_list[j].x_f << " landm.y_f= " << map_landmarks.landmark_list[j].y_f << " dst = " << dst << endl;
				distances.push_back(dst);
				}

			// find the shortest distance
			vector<double>::iterator result = min_element(begin(distances), end(distances));
			Map::single_landmark_s lm = map_landmarks.landmark_list[distance(begin(distances), result)];

			// multivariate gaussian probability
			float x = obs_tf.x; 
			float y = obs_tf.y; 
			float ux = map_landmarks.landmark_list[lm.id_i-1].x_f; ;
			float uy = map_landmarks.landmark_list[lm.id_i-1].y_f; ;
			//cout << x << " " << y << " " << ux << " " << " " << uy << endl;
			//cout << "distance to closets landmark= " << sqrt((x-ux)*(x-ux)+(y-uy)*(y-uy)) << endl;	
			float Pa = 1.0/(2.0 * M_PI * std_landmark[0] * std_landmark[1]);
			float Pb = exp(- ( ((x-ux)*(x-ux))/(2*std_landmark[0]*std_landmark[0]) + ((y-uy)*(y-uy))/(2*std_landmark[1]*std_landmark[1])));
			float P = Pa * Pb;
			//cout << "Pa = " << Pa << " Pb = " << Pb << " P = " << P << endl;	
			prob *= P;	
			//cout << "p*p*p*p....= " << prob << endl;
		}		

	// update weights
	weights[i] = prob;
	particles[i].weight = prob;
	//cout << "i= " << i << "	weights[i]=" << weights[i] << " particles[i].weight= " << particles[i].weight << endl;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    
    std::vector<Particle> tmp_particles;
	// random discrete distribution based on weights
	std::discrete_distribution<> d(weights.begin(), weights.end());

    default_random_engine gen;
	
    for (int i = 0; i < num_particles; ++i) {
		// randomly pick a particle based on its weight
        int index = d(gen);
		// add that particle to a new list
        tmp_particles.push_back(particles[index]);
    }

	// resampled particles based on previous weight
    particles = tmp_particles;	
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
