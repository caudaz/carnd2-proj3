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
	}
	// initialize flag
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	for (int i=0; i < num_particles; i++){	
		particles[i].x     = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
		particles[i].y     = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;

/* 		std::default_random_engine generator;
		std::normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		std::normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		std::normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);	
		particles[i].x     = dist_x(generator);
		particles[i].y     = dist_y(generator);
		particles[i].theta = dist_theta(generator);	 */	

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

for (int i = 0; i < particles.size(); i++){

	float prob = 1.0;

	// new vector to store transformed observations
	std::vector<LandmarkObs> observations_transf; 

	// iterator for obsrv   Output = transformed observations into maps global c.s.
	for (LandmarkObs obsrv : observations){
		LandmarkObs obs_tf;
		obs_tf.id = obsrv.id;
		obs_tf.x = particles[i].x + cos(particles[i].theta)*obsrv.x + sin(particles[i].theta)*obsrv.y;
		obs_tf.y = particles[i].y - sin(particles[i].theta)*obsrv.x + cos(particles[i].theta)*obsrv.y;
		//observations_transf.push_back(obs_tf);		

		// distances for each observation
		vector <double> distances;

		// iterator for landm
		for (Map::single_landmark_s landm : map_landmarks.landmark_list) {
							double dst = dist(landm.x_f, landm.y_f, obs_tf.x, obs_tf.y);
							// contains : distances to all landmarks
							distances.push_back(dst);
		}

		vector<double>::iterator result = min_element(begin(distances), end(distances));
		Map::single_landmark_s lm = map_landmarks.landmark_list[distance(begin(distances), result)];
		// this observation is closest to this landmark id
		obs_tf.id = lm.id_i;
		// contains : 1-closest landmark id, 2-transf x-loc , 3-transf y-loc
		observations_transf.push_back(obs_tf);

	float x = obs_tf.x;
	float y = obs_tf.y;
	float ux = map_landmarks.landmark_list[lm.id_i].x_f;
	float uy = map_landmarks.landmark_list[lm.id_i].y_f;	
	float Pa = 1/(2 * M_PI * std_landmark[0] * std_landmark[1]);
	float Pb = exp(- (pow(x-ux,2)/(2*pow(std_landmark[0],2)) + pow(y-uy,2)/(2*pow(std_landmark[1],2))));
	float P = Pa * Pb;
	
	prob *= P;
	
	}		

	particles[i].weight = prob;	
	cout << "i= " << i << "particles[i].weight = " << particles[i].weight << endl;

	/* 
	def P(x,y,sx,sy,ux,uy):
	P=1/(2*math.pi*sx*sy) * math.exp(- ( ((x-ux)**2)/(2*sx**2) + ((y-uy)**2)/(2*sy**2)))
	Pobs1 = P(xtobs1,ytobs1,0.3,0.3,L[0][0],L[0][1])
	Pobs2 = P(xtobs2,ytobs2,0.3,0.3,L[1][0],L[1][1])
	Pobs3 = P(xtobs3,ytobs3,0.3,0.3,L[1][0],L[1][1])
	Part_final_weight = Pobs1 * Pobs2 * Pobs3 
	*/
		
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
