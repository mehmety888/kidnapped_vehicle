
#include "particle_filter.h"

#include <cmath>
#include <algorithm>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 100;  // Set the number of particles
  particles.resize(num_particles);
  std::default_random_engine gen;
  // Create normal (Gaussian) distributions for x, y and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  for (unsigned int index = 0; index < (unsigned int)num_particles; ++index)
  {
    particles[index].id = index;
    particles[index].x = dist_x(gen);
    particles[index].y = dist_y(gen);
    particles[index].theta = dist_theta(gen);
    particles[index].weight = 1.0;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  double x_f = 0.0;
  double y_f = 0.0;
  double theta_f = 0.0;
  std::default_random_engine motion_gen;

  for (unsigned int index = 0; index < (unsigned int)num_particles; ++index)
  {
    // using bicycle model with yaw rate is not equal to zero to get final position 
    x_f = particles[index].x + (velocity / yaw_rate) * (sin(particles[index].theta + yaw_rate * delta_t) - sin(particles[index].theta));
    y_f = particles[index].y + (velocity / yaw_rate) * (cos(particles[index].theta) - cos(particles[index].theta + yaw_rate * delta_t));
    theta_f = particles[index].theta + yaw_rate * delta_t;
    
    // Create normal (Gaussian) distributions for x_f, y_f and theta_f
    std::normal_distribution<double> dist_x_f(x_f, std_pos[0]);
    std::normal_distribution<double> dist_y_f(y_f, std_pos[1]);
    std::normal_distribution<double> dist_theta_f(theta_f, std_pos[2]);
    
    particles[index].x = dist_x_f(motion_gen);
    particles[index].y = dist_y_f(motion_gen);
    particles[index].theta = dist_theta_f(motion_gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  for (unsigned int i = 0; i < (unsigned int)observations.size(); ++i)
  {
    double min_distance = std::numeric_limits<double>::infinity();
    unsigned int temp_id = (unsigned int)predicted.size();
    for (unsigned int j = 0; j < (unsigned int)predicted.size(); ++j)
    {
      double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if (distance < min_distance)
      {
        min_distance = distance;
        temp_id = predicted[j].id;
      }
    }
    observations[i].id = temp_id;
  }
}
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  double total_weight = 0.0;
  for (unsigned int i = 0; i < (unsigned int)particles.size(); ++i)
  {
    // coordinate transformation from vehicle coordinate to map coordinate
    vector<LandmarkObs> observations_mapcoordinate(observations.size());
    for (unsigned int j = 0; j < (unsigned int)observations.size(); ++j)
    {
      observations_mapcoordinate[j] = LandmarkObs{observations[j].id,
                                                  observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta)+                                                   particles[i].x,
                                                  observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta)+                                                   particles[i].y};
    }
    // find landmarks in sensor range
    vector<LandmarkObs> landmarks_inrange;
    landmarks_inrange.reserve(map_landmarks.landmark_list.size());
    for (unsigned int k = 0; k < (unsigned int)map_landmarks.landmark_list.size(); ++k)
    {
      if(sensor_range > dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f))
      {
       landmarks_inrange.push_back(LandmarkObs{map_landmarks.landmark_list[k].id_i,
                                               map_landmarks.landmark_list[k].x_f,
                                               map_landmarks.landmark_list[k].y_f});
      }
    }
    dataAssociation(landmarks_inrange, observations_mapcoordinate);
    // multivariate gaussian distribution to find weights
    for (unsigned int l = 0; l < (unsigned int)landmarks_inrange.size(); ++l)
    {
      for (unsigned int m = 0; m < (unsigned int)observations_mapcoordinate.size(); ++m)
      {
        if(landmarks_inrange[l].id == observations_mapcoordinate[m].id)
        {
          particles[i].weight *= (1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1])) *
                                 exp(-1.0 * ((pow((observations_mapcoordinate[m].x - landmarks_inrange[l].x), 2) /
                                 (2.0 * std_landmark[0] * std_landmark[0])) + 
                                 (pow((observations_mapcoordinate[m].y - landmarks_inrange[l].y), 2) /
                                 (2.0 * std_landmark[1] * std_landmark[1]))));
        }
      }
    }
    total_weight += particles[i].weight;
  }
  // normalize weights
  for (unsigned int n = 0; n < (unsigned int)particles.size(); ++n)
  {
    particles[n].weight = particles[n].weight / total_weight;
  }
}

void ParticleFilter::resample() {
  vector<Particle> particles_sampled (particles.size());
  // create weights vector to use in discrete_distribution
  vector<double> weights(particles.size());
  for (unsigned int i = 0; i < (unsigned int)particles.size(); ++i)
  {
    weights[i] = particles[i].weight;
  }
  // resample
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(),weights.end());
  for(unsigned int j = 0; j < (unsigned int)particles.size(); ++j)
  {      
    particles_sampled[j] = particles[d(gen)];
  }
  particles = particles_sampled;
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
