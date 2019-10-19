/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 30;  // TODO: Set the number of particles
  
  std::default_random_engine gen;
  // Gaussian distribution for x, y, and theta
  normal_distribution<double> p_x(x, std[0]);
  normal_distribution<double> p_y(y, std[1]);
  normal_distribution<double> p_theta(theta, std[2]);
  // generate all particles.
  for (int i=0;i<num_particles;i++) {
    Particle p;
    p.id = i;
    p.x = p_x(gen);
    p.y = p_y(gen);
    p.theta = p_theta(gen);
    p.weight = 1;
    particles.push_back(p);
  }
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  // Gaussian distribution for x, y, and theta
  normal_distribution<double> p_x(0.0, std_pos[0]);
  normal_distribution<double> p_y(0.0, std_pos[1]);
  normal_distribution<double> p_theta(0.0, std_pos[2]);
  // update each particle position and heading, plus noises
  // yaw rate != 0
  if (fabs(yaw_rate) > 0.0001) {
	  for (int i=0;i<num_particles;i++) {
		double theta = particles[i].theta;
		particles[i].x += velocity/yaw_rate*(sin(theta + yaw_rate * delta_t) - sin(theta)) + p_x(gen);
		particles[i].y += velocity/yaw_rate*(cos(theta) - cos(theta + yaw_rate * delta_t)) + p_y(gen);
		particles[i].theta += yaw_rate * delta_t + p_theta(gen);
	  }
  // yaw rate == 0
  } else {
	  for (int i=0;i<num_particles;i++) {
		double theta = particles[i].theta;
		particles[i].x += velocity*cos(theta)*delta_t + p_x(gen);
		particles[i].y += velocity*sin(theta)*delta_t + p_y(gen);
		particles[i].theta += yaw_rate * delta_t + p_theta(gen);
	  }
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // loop through all obs landmarks
  for (unsigned int i=0;i<observations.size();i++) {
    double dis = std::numeric_limits<double>::infinity();
	int index = 0;
    // loop through all predicted measurements
    for (unsigned int j=0;j<predicted.size();j++) {
      double temp = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (temp < dis) {
        dis = temp;
        index = j;
      }
    }
	// update obs landmark id
    observations[i].id = predicted[index].id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  weights.clear();
  // loop through particles
  for (int i=0;i<num_particles;i++) {
    vector<LandmarkObs> Obs_t_list; // list of observed landmarks transformed to map coord
    vector<LandmarkObs> lmk_in_range_list; // list of landmarks within onboard sensor range
    //loop through observed landmarks
    for (unsigned int j=0;j<observations.size();j++) {
      // transform to map coordinates
      LandmarkObs Obs_t; // observed landmark transformed to map coord
      Obs_t.x = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
      Obs_t.y = particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;
      Obs_t_list.push_back(Obs_t);
    }
    // loop through map lanmarks to select ones within sensor range
    for (unsigned int j=0;j<map_landmarks.landmark_list.size();j++) {
      LandmarkObs lmk_in_range; // landmark within onboard sensor range
      double lmk_dis = dist(map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f,particles[i].x,particles[i].y);
      if (lmk_dis < sensor_range) {
        lmk_in_range.x = map_landmarks.landmark_list[j].x_f;
        lmk_in_range.y = map_landmarks.landmark_list[j].y_f;
        lmk_in_range.id = map_landmarks.landmark_list[j].id_i;
        lmk_in_range_list.push_back(lmk_in_range);
      }
    }
    // associate each observation with a landmark id with method of nearest neighbor
    dataAssociation(lmk_in_range_list, Obs_t_list);

    // take product of all multivariate Gaussian probabilities (observations of landmarks) from one particle.
    double prob = 1.0;
    for (unsigned int j=0;j<Obs_t_list.size();j++) {
      
      double x = Obs_t_list[j].x;
      double y = Obs_t_list[j].y;
      double id = Obs_t_list[j].id;
      LandmarkObs lmk;
      // matching observed landmark id to landmark within sensor range to extract the position in map coordinates.
      for (unsigned int k=0;k<lmk_in_range_list.size();k++) {
        if (id == lmk_in_range_list[k].id)
          lmk = lmk_in_range_list[k];
      }
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double exp_term = -((x-lmk.x)*(x-lmk.x)/(2*sig_x*sig_x) + (y-lmk.y)*(y-lmk.y)/(2*sig_y*sig_y));
      prob = prob * 1/(2*M_PI*sig_x*sig_y)*exp(exp_term);
    }
    particles[i].weight = prob;
    weights.push_back(prob);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  // use the resampling wheel example concept in the lecture
  double beta = 0.0;
  vector<Particle> p1;
  std::default_random_engine gen;
  std::uniform_int_distribution<int> distribution(0,num_particles-1);
  std::uniform_real_distribution<double> real_n(0.0,1.0);
  double max_w = *max_element(weights.begin(),weights.end());
  for (int i=0;i<num_particles;i++) {
    int index = distribution(gen);
    beta += 2 * real_n(gen) * max_w;
    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1)%num_particles;
    }
    p1.push_back(particles[index]);
  }
  particles = p1;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}