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

  // set the number of particles
  num_particles = 1000;

  // create a random generator for sampling
  default_random_engine gen;

  // Standard deviations for x, y, and theta
  double std_x, std_y, std_theta;

  // DEBUG params
  double u_x = 0;
  double u_y = 0;
  double u_theta = 0;

  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];


  // This line creates a normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // initialise particles
  for (int i = 0; i < num_particles; ++i) {
    // sample from distribution to initialise particles and set weight to 1
    Particle particle;

    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(1);

    // debug
    u_x += particle.x;
    u_y += particle.y;
    u_theta += particle.theta;
  }

  // write out some log for debug
  cout << "initialised: " << particles.size() << endl;
  cout << "x " << x << endl;
  cout << "y " << y << endl;
  cout << "theta " << theta << endl;
  cout << "first particle: " << particles[50].id << endl;
  cout << "first particle: " << particles[50].x << endl;
  cout << "first particle: " << particles[50].y << endl;
  cout << "first particle: " << particles[50].theta << endl;
  cout << "first particle: " << particles[50].weight << endl;
  // additional DEBUG
  cout << "mean x: " << u_x << endl;
  cout << "mean y: " << u_y << endl;
  cout << "mean theta: " << u_theta << endl;

  // set initialised as true...
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // create engines / noise gen for x, y, theta
  default_random_engine gen;

  // Standard deviations for x, y, and theta
  double std_x, std_y, std_theta;

  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];

  // create noise distribution for each sensor input's std-dev
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // loop through each particle and predict new x, y, theta + noise and update
  for (int i = 0; i < num_particles; ++i) {
    // x = x-1 + v/yaw * [sin(theta-1 + yaw * dt) - sin(theta-1)] + noise
    particles[i].x = particles[i].x +
                     (velocity/yaw_rate)*
                     (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta))
                     + dist_x(gen);

    // y = y-1 + v/yaw * [-cos(theta-1 + yaw * dt) + cos(theta-1)] + noise
    particles[i].y = particles[i].y +
                     (velocity/yaw_rate)*
                     (-cos(particles[i].theta + yaw_rate*delta_t) + cos(particles[i].theta))
                     + dist_y(gen);

    // theta = theta-1 + yaw * dt + noise
    particles[i].theta = particles[i].theta + yaw_rate*delta_t + dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


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

  // for each particle...
  for (int i = 0; i < num_particles; ++i) {
    // transform to map coords using homogenous transformation
    // x_map= x_part + (np.cos(theta) * x_obs) - (np.sin(theta) * y_obs)
    // y_map= y_part + (np.sin(theta) * x_obs) + (np.cos(theta) * y_obs)
    std::vector<LandmarkObs> transformed_observations = observations;
    for (int j = 0; j < transformed_observations.size(); ++j) {
      transformed_observations[j].x = particles[i].x +
                                      (cos(particles[i].theta) * transformed_observations[j].x) -
                                      (sin(particles[i].theta) * transformed_observations[j].y);
      transformed_observations[j].y = particles[i].y +
                                      (sin(particles[i].theta) * transformed_observations[j].x) +
                                      (cos(particles[i].theta) * transformed_observations[j].y);
    }
    // now associate each observation with a landmark using euclidean distances
    // this gets us the mu_x and mu_y for each observation
    // convert map to Obs struct
    double best_distance = 10000;
    int matching_id = 0;
    for (int z = 0; z < transformed_observations.size(); ++z) {
      for (int y = 0; y < map_landmarks.landmark_list.size(); ++y) {
        if (y == 0){
          best_distance = dist(map_landmarks.landmark_list[y].x_f,map_landmarks.landmark_list[y].y_f,transformed_observations[z].x,transformed_observations[z].y);
          matching_id = map_landmarks.landmark_list[y].id_i;
        } else {
          if (dist(map_landmarks.landmark_list[y].x_f,map_landmarks.landmark_list[y].y_f,transformed_observations[z].x,transformed_observations[z].y) < best_distance){
            best_distance = dist(map_landmarks.landmark_list[y].x_f,map_landmarks.landmark_list[y].y_f,transformed_observations[z].x,transformed_observations[z].y);
            matching_id = map_landmarks.landmark_list[y].id_i;
          }
        }
      }
      transformed_observations[z].id = map_landmarks.landmark_list[matching_id].id_i;
    }
    // calculate the updated weight in 2 steps
    double probability_sum = 1;
    double sig_x, sig_y;
    sig_x = std_landmark[0];
    sig_y = std_landmark[1];
    double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));
    for (int j = 0; j < transformed_observations.size(); ++j) {
      int map_id = transformed_observations[j].id;
      double exponent = ((transformed_observations[j].x - map_landmarks.landmark_list[map_id].x_f)*(transformed_observations[j].x - map_landmarks.landmark_list[map_id].x_f))/(2 * sig_x*sig_x) +
                        ((transformed_observations[j].y - map_landmarks.landmark_list[map_id].y_f)*(transformed_observations[j].y - map_landmarks.landmark_list[map_id].y_f))/(2 * sig_y*sig_y);
      double weight = gauss_norm * exp(-exponent);
      probability_sum *= weight;
    }
    particles[i].weight = probability_sum;
    weights[i] = probability_sum;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // set up engine
  default_random_engine gen;
  std::discrete_distribution<int> distribution(0, num_particles);

  // create temp variables
  std::vector<Particle> t_particles;
  int p_index = distribution(gen);
  double beta = 0.0;
  double max_weight = 0.0;
  for (int j = 0; j < particles.size(); ++j) {
    if (weights[j] > max_weight) {
      max_weight = weights[j];
    }
  }
  for (int j = 0; j < particles.size(); ++j) {
    beta += (rand()/RAND_MAX) * 2 * max_weight;
    while (beta > particles[p_index].weight) {
      beta -= particles[p_index].weight;
      p_index = (p_index + 1) % particles.size();
    }
    t_particles.push_back(particles[p_index]);
  particles = t_particles;
  }
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
