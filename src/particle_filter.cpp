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
  num_particles = 100;

  // create a random generator for sampling
  default_random_engine gen;

  // Standard deviations for x, y, and theta
  double std_x, std_y, std_theta;

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
    weights.push_back(particle.weight);
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
    if (fabs(yaw_rate) < 0.0001){
      // x = x-1 + v/yaw * [sin(theta-1 + yaw * dt) - sin(theta-1)] + noise
      particles[i].x += particles[i].x +
                       (velocity*delta_t)*cos(particles[i].theta) +
                       dist_x(gen);

      // y = y-1 + v/yaw * [-cos(theta-1 + yaw * dt) + cos(theta-1)] + noise
      particles[i].y = particles[i].y +
                       (velocity*delta_t)*sin(particles[i].theta) +
                       dist_y(gen) ;

      // theta = theta as no yaw rate
      particles[i].theta = particles[i].theta + dist_theta(gen) ;
    } else {
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
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
  for (unsigned int i = 0; i < observations.size(); ++i) {

    // observation
    LandmarkObs obs = observations[i];

    // initialise a distance to max possible
    double min_distance = numeric_limits<double>::max();

    // initialise index to impossible value
    int map_idx = -1;

    for (unsigned int j = 0; j < predicted.size(); ++j) {
      // get prediction
      LandmarkObs pred = predicted[j];

      // get distance between observation and prediction landmarks
      double distance = dist(obs.x, obs.y, pred.x, pred.y);

      // find the predicted landmark nearest the current observed landmark
      if (distance < min_distance) {
        min_distance = distance;
        map_idx = pred.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    observations[i].id = map_idx;
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

  // for each particle...
  for (int i = 0; i < num_particles; ++i) {

    // particle values to help readability
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    // create a vector to hold particle's view of observations
    vector<LandmarkObs> predictions;

    // for each map landmark...
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) {

      // get landmark values
      float lmark_x = map_landmarks.landmark_list[j].x_f;
      float lmark_y = map_landmarks.landmark_list[j].y_f;
      int lmark_id = map_landmarks.landmark_list[j].id_i;

      // use sensor range to weed out landmarks too far away to measure
      if (fabs(lmark_x - p_x) <= sensor_range && fabs(lmark_y - p_y) <= sensor_range) {

        // add prediction to vector
        predictions.push_back(LandmarkObs{ lmark_id, lmark_x, lmark_y });
      }
    }

    // transform observations to Map coords
    vector<LandmarkObs> transformed_obs;
    for (unsigned int j = 0; j < observations.size(); ++j) {
      double trans_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double trans_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
      transformed_obs.push_back(LandmarkObs{ observations[j].id, trans_x, trans_y });
    }

    // associate predictions (the map) with map coord observations
    dataAssociation(predictions, transformed_obs);

    // reset weight for use in update product calculation
    particles[i].weight = 1.0;

    // update weight based on observations
    for (unsigned int j = 0; j < transformed_obs.size(); ++j) {

      // variables for calculations
      double obs_x, obs_y, pred_x, pred_y;
      obs_x = transformed_obs[j].x;
      obs_y = transformed_obs[j].y;

      int landmark_id = transformed_obs[j].id;

      // get landmark
      for (unsigned int k = 0; k < predictions.size(); ++k) {
        if (predictions[k].id == landmark_id) {
          pred_x = predictions[k].x;
          pred_y = predictions[k].y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double sigma_x = std_landmark[0];
      double sigma_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*sigma_x*sigma_y)) * exp( -( pow(pred_x-obs_x,2)/(2*pow(sigma_x, 2)) + (pow(pred_y-obs_y,2)/(2*pow(sigma_y, 2))) ) );

      // product of this obersvation weight with total observations weight
      particles[i].weight *= obs_w;
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<Particle> new_particles;

  // get weights
  vector<double> weights;
  for (int i = 0; i < num_particles; ++i) {
    weights.push_back(particles[i].weight);
  }

  // get a random starting index for the wheel
  uniform_int_distribution<int> uid(0, num_particles-1);
  default_random_engine gen;
  auto index = uid(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> urd(0.0, max_weight);

  double beta = 0.0;

  // go round the wheel
  for (int i = 0; i < num_particles; i++) {
    beta += urd(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
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
