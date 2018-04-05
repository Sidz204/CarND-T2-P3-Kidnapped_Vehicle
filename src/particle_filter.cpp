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
#include <cfloat> //required for getting max value of double in C++

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	


	num_particles =100; //set no of particles
	default_random_engine gen; //define random no generation engine

	// resize weights and particles vector
  	weights.resize(num_particles);
	particles.resize(num_particles);

	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		
		
		// Sample  and from these normal distrubtions
		// where "gen" is the random engine initialized earlier.
 		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;

	}

	is_initialized = true;

}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	//randomly generating noise
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0,std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);


	//predict the vehicle position and orientation using formulaes of motion model
	for(int i=0; i<num_particles; ++i){
		if(fabs(yaw_rate)<0.00001)
		{
			particles[i].x+= velocity*delta_t*cos(particles[i].theta);
			particles[i].y+= velocity*delta_t*sin(particles[i].theta);

		}
		else
		{
			particles[i].x+=(velocity/yaw_rate)*(sin(particles[i].theta + (delta_t*yaw_rate))-sin(particles[i].theta));
			particles[i].y+=(velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta + (delta_t*yaw_rate)));
			particles[i].theta+=yaw_rate*delta_t;
		}

		//adding noise
		particles[i].x+=dist_x(gen);
		particles[i].y+=dist_y(gen);
		particles[i].theta+=dist_theta(gen);
		//cout <<particles[i].x<<endl;
	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

	for (int i=0;i<observations.size();++i)
	{
		
		double min_dist = DBL_MAX; //max value of double in c++
		int close_particle_id = -1;
		for (int j=0;j<predicted.size();++j)
		{
			double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y); //distance between observation & landmark
			if (distance<min_dist)
			{
				min_dist = distance;
				close_particle_id = j;
			}
		}

		observations[i].id = close_particle_id; //assigning nearest landmark id to the observation id
		
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	
	for(int i=0;i<num_particles;++i)
	{
	
		//consider landmarks within sensor range
		vector<LandmarkObs> lm_inrange; 
		for(int j=0;j<map_landmarks.landmark_list.size();++j)
		{

			double distance = dist(map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f,particles[i].x,particles[i].y);
			if(distance<=sensor_range)
			{
				lm_inrange.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
			}

		}

		if (lm_inrange.size()==0)
		{
			particles[i].weight = 0; //if landmarks not in range assign 0 to weights
			weights[i] = 0;
		}
		else
		{
		
			//step1 : transformation into map coordinates using homogenous transformation 
			vector<LandmarkObs> transformed_obs;
			for(int k=0;k < observations.size();++k)
			{
				
				double xm,ym;
				xm = particles[i].x + (cos(particles[i].theta)*observations[k].x) - (sin(particles[i].theta)*observations[k].y);
				ym = particles[i].y + (sin(particles[i].theta)*observations[k].x) + (cos(particles[i].theta)*observations[k].y);
				transformed_obs.push_back(LandmarkObs{observations[k].id,xm,ym});
			}
		
			//step2 : associate using nearest neighbour
			dataAssociation(lm_inrange,transformed_obs);

			

			//step3 : calculate weight using gaussian multi-variate probability

			double weight_prod = 1.0;
			double sig_x= std_landmark[0];
			double sig_y= std_landmark[1];
			// calculate normalization term
			double gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
			
			for(int m=0;m< transformed_obs.size();++m)
			{

				
				double x_obs= transformed_obs[m].x;
				double y_obs= transformed_obs[m].y;
				auto mu_x= lm_inrange[transformed_obs[m].id].x;
				auto mu_y= lm_inrange[transformed_obs[m].id].y;

				// calculate exponent
				double exponent= (pow((x_obs - mu_x),2)/(2 * pow(sig_x,2))) + (pow((y_obs - mu_y),2)/(2 * pow(sig_y,2)));

				// calculate total weight using normalization terms and exponent
				weight_prod += gauss_norm * exp(-exponent);


			}

			particles[i].weight = weight_prod; //updating weights
			weights[i] = weight_prod;

		}

	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	vector<Particle> new_particles(num_particles);
  	default_random_engine gen;

  	//discrete distribution decides probability on the basis of the weights provided and returns that index position 
  	discrete_distribution<int> dist_weights(weights.begin(), weights.end());

  	for (int i = 0; i < num_particles; ++i) 
  	{
    new_particles[i] = particles[dist_weights(gen)];
  	}

  	// replace old particles with new ones by using std::move to avoid deep copy .
	particles = move(new_particles);

}



Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
