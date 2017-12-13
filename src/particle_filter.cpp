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
#include<random>
#include <limits>
#include <algorithm>
#include <fstream>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	// This line creates a normal (Gaussian) distribution for x.
	is_initialized=false;
	default_random_engine gen;
//	cout<<"*****************"<<endl;
	normal_distribution<double> dist_x(x, std[0]);
//	cout<<"*****************"<<endl;

	// TODO: Create normal distributions for y and theta.
	normal_distribution<double> dist_y(y,std[1]);
	normal_distribution<double> dist_theta(theta,std[2]);
	num_particles=10;
	for (int p=0; p<num_particles;p++){
		Particle temp_particle;
		temp_particle.id=p;
		temp_particle.x=dist_x(gen);
		temp_particle.y=dist_y(gen);
		temp_particle.theta= dist_theta(gen);
		temp_particle.weight=1.0;
		particles.push_back(temp_particle);


	}
	is_initialized=true;
	cout<<"intailised"<<endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
//	cout<<"start prediction"<<endl;
	default_random_engine gen;
//	double std_velocity=.5;
//	double std_yawr=.5;
//	normal_distribution<double> dist_v(0, std_velocity);
//	normal_distribution<double> dist_yawr(0,std_yawr);
//	velocity += dist_v(gen);
//	yaw_rate +=dist_yawr(gen);
	ofstream update_out2;
	update_out2.open("Particles.txt");
	if (!update_out2.is_open())
			{
//		  cout << "Unable to open file for writing";

			}

	update_out2<<"start prediction++++++++++++++++"<<endl;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0,std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);





	for (int p=0; p<num_particles;p++){



		if (fabs(yaw_rate)>.0001){
			particles[p].x= particles[p].x+ (velocity/yaw_rate)*(sin(particles[p].theta+yaw_rate*delta_t)-sin(particles[p].theta));
			particles[p].y= particles[p].y+ (velocity/yaw_rate)*(cos(particles[p].theta)-cos(particles[p].theta+yaw_rate*delta_t));

		}
		else{
			particles[p].x= particles[p].x+ velocity*cos(particles[p].theta)*delta_t;
			particles[p].y= particles[p].y+ velocity*sin(particles[p].theta)*delta_t;

		}
		particles[p].theta= particles[p].theta+ yaw_rate*delta_t;
		particles[p].x =particles[p].x+ dist_x(gen);
		particles[p].y =particles[p].y+dist_y(gen);
		particles[p].theta = particles[p].theta+dist_theta(gen);
//		update_out2<<"Particle number= "<<p<<endl;
//
//		update_out2<<"Particle X= "<<particles[p].x<<"  Particle y="<<particles[p].y<<endl;


	}
//	update_out2<<"end prediction++++++++++++++++"<<endl;
//	cout<<"end prediction"<<endl;

}

int ParticleFilter::dataAssociation(Map mapLandmark,LandmarkObs observation) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
//	cout<<"start asso"<<endl;
//	cout <<"landmark list"<<mapLandmark.landmark_list.size()<<endl;
	double distance= std::numeric_limits<double>::max();
	int id;
	for (int lm=0; lm<mapLandmark.landmark_list.size();lm++){
		double temp_dist= dist(observation.x,observation.y,mapLandmark.landmark_list[lm].x_f,mapLandmark.landmark_list[lm].y_f);
		if (temp_dist<distance){
			distance=temp_dist;
			id= mapLandmark.landmark_list[lm].id_i;
//			cout<<"ID"<<id<<endl;
		}

	}
	return id;

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
	ofstream update_out;
	ofstream update_out2;
		update_out2.open("update.txt");
		if (!update_out2.is_open())
				{
			  cout << "Unable to open file for writing";

				}
		update_out2<<"status Update"<<endl;

	update_out.open("update_out.txt");
	if (!update_out.is_open())
	        {
	  	  cout << "Unable to open file for writing";

	        }
	cout<<"start update"<<endl;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
	double total_weight=0.0;
//	update_out<<"number of observation= "<<observations.size()<<endl;

	for (int p=0; p<num_particles;p++){
		update_out<<"Particle number= "<<p<<endl;

		Particle current_particle= particles[p];
		double p_x= current_particle.x;
		double p_y=current_particle.y;
		current_particle.weight=1.0;
		LandmarkObs transformed_lm;
		vector <LandmarkObs> transformedList_lm;
		Map mapLandList;
		for (int l=0; l< map_landmarks.landmark_list.size();l++){
			float l_x=map_landmarks.landmark_list[l].x_f;
			float l_y=map_landmarks.landmark_list[l].y_f;
			int l_id=map_landmarks.landmark_list[l].id_i;
			if (fabs(l_x-p_x)<=sensor_range && fabs(l_y-p_y)<=sensor_range){
				Map::single_landmark_s single_lm{l_id,l_x,l_y};
				mapLandList.landmark_list.push_back(single_lm);
				update_out<<"predicted map= "<<single_lm.id_i<<endl;
			}
		}
//		update_out<<"size predicted lanmark= "<<mapLandList.landmark_list.size()<<endl;
		for(int land=0; land<mapLandList.landmark_list.size();land++){
			update_out<<"map ID= "<<mapLandList.landmark_list[land].id_i<<"map X= "<<mapLandList.landmark_list[land].x_f
					<<"map Y= "<<mapLandList.landmark_list[land].y_f<<endl;
		}

//		cout<<"start update2"<<endl;
//		update_out<<"intital current weight= "<<current_particle.weight<<endl;

		for (int obv=0; obv<observations.size();obv++){
			transformed_lm.x=cos(current_particle.theta)*observations[obv].x-sin(current_particle.theta)*observations[obv].y+current_particle.x*1;
			transformed_lm.y=sin(current_particle.theta)*observations[obv].x+cos(current_particle.theta)*observations[obv].y+current_particle.y*1;
			transformed_lm.id= dataAssociation(mapLandList,transformed_lm);

			update_out<<" observerd x= "<<observations[obv].x<<"  observed y="<<observations[obv].y<<endl;
			update_out<<"Particle X= "<<particles[p].x<<"  Particle y="<<particles[p].y<<endl;
			update_out<<"transformed id= "<<transformed_lm.id<<" transformed x= "<<transformed_lm.x<<"  transformed y="<<transformed_lm.y<<endl;

			transformedList_lm.push_back(transformed_lm);
			associations.push_back(transformed_lm.id);
		    sense_x.push_back(transformed_lm.x);
		    sense_y.push_back(transformed_lm.y);

		    update_out<<"map id= "<<map_landmarks.landmark_list[transformed_lm.id-1].id_i<<endl;

		    double weight=pow((transformed_lm.x-map_landmarks.landmark_list[transformed_lm.id-1].x_f),2)/(2*pow(std_landmark[0],2))
						+ (pow((transformed_lm.y-map_landmarks.landmark_list[transformed_lm.id-1].y_f),2)/(2*pow(std_landmark[1],2)));
			double gauss_norm=1/(2*M_PI*std_landmark[0]*std_landmark[1]);
			double prob_weight= gauss_norm*exp(-weight);

			update_out<<"stdx= "<<std_landmark[0]<<"   stdy= "<<std_landmark[1]<<endl;
			current_particle.weight *=prob_weight;


//			update_out<<" weight= "<<weight<<endl;
//			update_out<<" gaussian norm= "<<gauss_norm<<endl;
//			update_out<<"prob weight= "<<prob_weight<<endl;
//			update_out<<"current weight= "<<current_particle.weight<<endl;

		}

		total_weight+=current_particle.weight;
		current_particle = SetAssociations(current_particle, associations, sense_x, sense_y);
//		cout<<"start update3"<<endl;
		particles[p]=current_particle;
//		update_out<<"End Updat particle***************************"<<endl;




	}
	update_out<<"End Update***************************"<<endl;

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
//	cout<<"start resample"<<endl;
	vector<double> weight_vector;
	vector<Particle> temp_particles;
	for (int p=0; p<particles.size();p++){
		weight_vector.push_back(particles[p].weight);
	}
	default_random_engine gen;
	discrete_distribution<int> d(weight_vector.begin(),weight_vector.end());

	int index= d(gen);

	double beta=0.0;

	double max_w= *max_element(weight_vector.begin(),weight_vector.end());
//	cout<<"end max w="<<max_w<<endl;
	uniform_real_distribution<double> unifor_v(0, 2*max_w);
	for (int p=0; p< num_particles;p++){
		Particle particle=particles[d(gen)];
		temp_particles.push_back(particle);
	}

//	for (int p=0; p<particles.size();p++){
//			beta+=unifor_v(gen);
//			while(beta >weight_vector[index]){
//				beta -=weight_vector[index];
//				index=(index+1)%num_particles;
//			}
//			temp_particles.push_back(particles[index]);
//		}
	particles=temp_particles;





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
