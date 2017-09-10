
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
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  normal_distribution<double> normX(x, std[0]);
	normal_distribution<double> normY(y, std[1]);
	normal_distribution<double> normTheta(theta, std[2]);
	weights.resize(num_particles, 1.0);
  for (int i = 0; i < num_particles; ++i){
		particles.push_back(Particle(i, normX(gen), normY(gen), normTheta(gen), 1.0));
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	normal_distribution<double> noiseX(0, std_pos[0]);
	normal_distribution<double> noiseY(0, std_pos[1]);
	normal_distribution<double> noiseTheta(0, std_pos[2]);

	for (int i=0; i<particles.size(); ++i){
		double px = particles[i].x;
		double py = particles[i].y;
		double theta = particles[i].theta;

		// predicted x, y, theta
		double pred_x = 0.0;
		double pred_y = 0.0;
    double pred_theta = 0.0;

    if (fabs(yaw_rate)>0.001){
			pred_x = px + velocity/yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			pred_y = py + velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
		}
		else{
			pred_x = px + velocity * delta_t * cos(theta);
			pred_y = py + velocity * delta_t * sin(theta);
		}

		pred_theta = theta + delta_t * yaw_rate;

    // add noise
		pred_x += noiseX(gen);
		pred_y += noiseY(gen);
		pred_theta += noiseTheta(gen);

		// update
		particles[i].x = pred_x;
		particles[i].y = pred_y;
		particles[i].theta = pred_theta;
	}
}

void ParticleFilter::localToGlobal(const Particle &p, vector<LandmarkObs> & locs){
	for (auto & loc: locs){
    double x = loc.x;
		double y = loc.y;
		loc.x = p.x + x * cos(p.theta) - y * sin(p.theta);
		loc.y = p.y + x * sin(p.theta) + y * cos(p.theta);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	for (auto & observation: observations){
		double minDist = numeric_limits<double>::max();
		for (int idx=0; idx<predicted.size(); ++idx) {
		  LandmarkObs &landmark = predicted[idx];
			double dq = distSquare(observation, landmark);
		  if (dq < minDist){
				minDist = dq;
				observation.id = idx;
			}
		}
	}
}
double ParticleFilter::gaussian2D(const LandmarkObs & obs, const LandmarkObs & landmark){
	const double sigX = 0.3;
	const double sigY = 0.3;
	double dx = obs.x - landmark.x;
	double dy = obs.y - landmark.y;
	return exp(-(dx*dx/(2*sigX*sigX) + dy*dy/(2*sigY*sigY)))/2.0*M_PI*sigX*sigY;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {

	double sq = sensor_range * sensor_range;

  for (int i=0; i<particles.size(); ++i){
		Particle &particle = particles[i];

		// collect landmarks in sensor_range of a particle
		vector<LandmarkObs> landmarksInRange;
		for (const auto & landmark: map_landmarks.landmark_list){
			double dq = distSquare(particle.x, particle.y, landmark.x_f, landmark.y_f);
			if (dq <= sq) {
				landmarksInRange.push_back(LandmarkObs(landmark.id_i, landmark.x_f, landmark.y_f));
			}
		}

		if (landmarksInRange.empty()){
			particle.weight = 0.0;
			weights[i] = 0.0;
			continue;
		}

    // convert observations to map coordinate
		vector<LandmarkObs> observationsInMap = observations;
    localToGlobal(particle, observationsInMap);

		// compute nearest-neighbors of observations
    dataAssociation(landmarksInRange, observationsInMap);

    double prob = 1;
		for (const auto & obs: observationsInMap){
      prob*=gaussian2D(obs, landmarksInRange[obs.id]);
		}
		particle.weight = prob;
		weights[i] = prob;
	}

}

void ParticleFilter::resample() {

  vector<Particle> resampled;
	uniform_int_distribution<int> unitInt(0, num_particles-1);
	int idx = unitInt(gen);
	double maxWeight = numeric_limits<double>::min();
	for (const auto & w: weights) maxWeight = max(w, maxWeight);
	// steps
	uniform_real_distribution<double> unitReal(0, maxWeight);
  double beta = 0;
	for (int i = 0; i < num_particles; ++i){
		beta += unitReal(gen)*2.0;
		while (beta > weights[idx]){
			beta -= weights[idx];
			idx = (idx + 1) % num_particles;
		}
		resampled.push_back(particles[idx]);
	}
  particles = resampled;
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
