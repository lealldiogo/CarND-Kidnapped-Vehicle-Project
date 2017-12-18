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
  default_random_engine gen;

  // This line creates a normal (Gaussian) distribution for x, y and theta.
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    // TODO: Sample  and from these normal distrubtions like this:
    // sample_x = dist_x(gen);
    // where "gen" is the random engine initialized earlier.
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(particle.weight);
  }

  is_initialized = true;
  /*

  CHECKOUT THE FORMULAS IN THE IMPLEMENTATION OF PARTICLE FILTER LESSON
  THERE IS ALSO SOME CODE IN PYTHON WHICH I CAN TRANSLATE TO C++

  */



}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;


  for (int i = 0; i < num_particles; ++i) {

    double new_x;
    double new_y;
    double new_theta;

    if (yaw_rate == 0) {
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta;
    } else {
      new_x = particles[i].x + velocity * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) / yaw_rate;
      new_y = particles[i].y + velocity * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) / yaw_rate;
      new_theta = particles[i].theta + yaw_rate * delta_t;
    }

    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);

  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.


  for (int j = 0; j < landmark_list.size(); ++j) {

  }



  for (int i = 0; i < num_particles; ++i) {

    vector<LandmarkObs> trans_observations;

    for (int o = 0; o < observations.size(); ++o) {
      LandmarkObs trans_obs;

      // transform to map x and y coordinates
      trans_obs.x = particles[i].x + (cos(particles[i].theta) * observations[o].x) - (sin(particles[i].theta) * observations[o].y)
      trans_obs.y = particles[i].y + (sin(particles[i].theta) * observations[o].x) + (cos(particles[i].theta) * observations[o].y)

      trans_observations.push_back(trans_obs)

    }

    particles[i].weight = 1.0;

    for (int t = 0; t < trans_observations.size(); ++t) {

      double landmark_x = map_landmarks.landmark_list[j].x_f
      double landmark_y = map_landmarks.landmark_list[j].y_f

      double closest_landmark;
      double dist = sqrt(pow(trans_observations[t].x - landmark_x,2.0)+pow(trans_observations[t].y - landmark_y,2.0));
      if (dist < closest_landmark) {
        closest_landmark = dist;
      }







      double mu_x = map_landmarks.landmark_list[association].x_f;
      double mu_y = map_landmarks.landmark_list[association].y_f;
      long double multiplier = 1/(2*M_PI*std_landmark[0]*std_landmark[1])
      if (multiplier > 0){
        particles[i].weight *= multiplier
      }

      associations.push_back.pushback(association + 1);
      sense_x.pushback(trans_observations[t].x);
      sense_y.pushback(trans_observations[t].y);

    }

  weights[i] = particles[i].weight;

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

}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
