/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 */

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>

//Messages needs
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "ros/ros.h"

#include "particle_filter.h"
#include "helper_functions.h"

ros::Publisher landmarks_pub;
// sensor_msgs::PointCloud landmarks;

using namespace std;



int main(int argc, char **argv) {
	

	//ros information
	ros::init(argc, argv, "mcl");
	ros::NodeHandle n;
	landmarks_pub = n.advertise<sensor_msgs::PointCloud>("landmarks", 10, true);
	ros::Publisher odometry_truth_pub = n.advertise<nav_msgs::Odometry>("odometry_truth", 10);
	ros::Publisher particle_best_pub = n.advertise<nav_msgs::Odometry>("prediction_ground", 10);
	ros::Publisher particlecloud_pub = n.advertise<geometry_msgs::PoseArray>("particlecloud", 10);
	ros::Publisher landmarks_ob_pub = n.advertise<sensor_msgs::PointCloud>("landmarks_ob", 10);

	ros::Rate loop(200);
	
	// parameters related to grading.
	int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	double max_runtime = 45; // Max allowable runtime to pass [sec]
	double max_translation_error = 1; // Max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // Max allowable yaw error [rad]



	// Start timer.
	int start = clock();
	
	//Set up parameters here
	double delta_t = 0.1; // Time elapsed between measurements [sec]
	double sensor_range = 50; // Sensor range [m]
	
	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
	double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> N_x_init(0, sigma_pos[0]);
	normal_distribution<double> N_y_init(0, sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;

	// ros::Publisher landmarks_pub;
	// Read map data
	Map map;
	if (!read_map_data("/home/jjj/ros_ws/src/Particle-Filter-master/data/map_data.txt", map)) {
		cout << "Error: Could not open map file" << endl;
		return -1;
	}
	
	sensor_msgs::PointCloud landmarks;
	ros::Time::init();
	landmarks.header.stamp = ros::Time::now();
	landmarks.header.frame_id = "map";
	landmarks.points.resize(map.landmark_list.size());
	for(int i = 0; i < map.landmark_list.size(); i++)
	{
		landmarks.points[i].x = map.landmark_list[i].x_f;
		landmarks.points[i].y = map.landmark_list[i].y_f;
		cout<<landmarks.points[i].x<<endl;
	}
	landmarks_pub.publish(landmarks);



	// Read position data
	vector<control_s> position_meas;
	if (!read_control_data("/home/jjj/ros_ws/src/Particle-Filter-master/data/control_data.txt", position_meas)) {
		cout << "Error: Could not open position/control measurement file" << endl;
		return -1;
	}
	
	// Read ground truth data
	vector<ground_truth> gt;
	if (!read_gt_data("/home/jjj/ros_ws/src/Particle-Filter-master/data/gt_data.txt", gt)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}
	//demonstrate the groud_truth
	// nav_msgs::Odometry odometry_truth;
    // odometry_truth.header.stamp = ros::Time::now();
    // odometry_truth.header.frame_id = "map";
	// for(int n = 0; n < gt.size(); n++)
	// {
    // 	odometry_truth.pose.pose.position.x = gt[n].x;
	// 	cout<<odometry_truth.pose.pose.position.x<<endl;
   	// 	odometry_truth.pose.pose.position.y = gt[n].y;
    // 	odometry_truth.pose.pose.position.z = 0.0;
    // 	geometry_msgs::Quaternion odom_quat_1 = tf::createQuaternionMsgFromYaw(gt[n].theta);
    // 	odometry_truth.pose.pose.orientation = odom_quat_1;
	// 	odometry_truth_pub.publish(odometry_truth);
	// 	//延迟一段时间，不然程序运行太快，发布不出来
	// 	loop.sleep();
	// }
	// odometry_truth_pub.publish(odometry_truth);
	
	// Run particle filter!
	int num_time_steps = position_meas.size();
	ParticleFilter pf;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	
	for (int i = 0; i < num_time_steps; ++i) {
		 //landmarks_pub.publish(landmarks);
		// cout << "Time step: " << i << endl;
		// Read in landmark observations for current time step.
		ostringstream file;
		file << "/home/jjj/ros_ws/src/Particle-Filter-master/data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
		vector<LandmarkObs> observations;
		if (!read_landmark_data(file.str(), observations)) {
			cout << "Error: Could not open observation file " << i+1 << endl;
			return -1;
		}
		//demonstrate the landmarks_ob
		// sensor_msgs::PointCloud landmarks_ob;
		// landmarks.header.stamp = ros::Time::now();
		// landmarks.header.frame_id = "map";
		// landmarks.points.resize(observations.size());
		// cout<<"报错点1"<<endl;
		// for(int p = 0; p < observations.size(); p++)
		// {
		// 	landmarks_ob.points[p].x = observations[p].x;
		// 	landmarks_ob.points[p].y = observations[p].y;
		// }
		// cout<<"报错点2"<<endl;
		// landmarks_ob_pub.publish(landmarks_ob);
		// loop.sleep();
		// cout<<"报错点3"<<endl;
		
		// Initialize particle filter if this is the first time step.
		if (!pf.initialized()) {
			n_x = N_x_init(gen);
			n_y = N_y_init(gen);
			n_theta = N_theta_init(gen);
			pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);
		}
		else {
			// Predict the vehicle's next state (noiseless).
			pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
		}
		// simulate the addition of noise to noiseless observation data.
		vector<LandmarkObs> noisy_observations;
		LandmarkObs obs;
		for (int j = 0; j < observations.size(); ++j) {
			n_x = N_obs_x(gen);
			n_y = N_obs_y(gen);
			obs = observations[j];
			obs.x = obs.x + n_x;
			obs.y = obs.y + n_y;
			noisy_observations.push_back(obs);
		}

		double temp_x = 0;
		double temp_y = 0;
		double temp_theta = 0;
		vector<double> ob_x;
		vector<double> ob_y;
		for(int i = 0; i < pf.particles.size(); i++)
		{
			temp_x += pf.particles[i].x;
			temp_y += pf.particles[i].y;
			temp_theta += pf.particles[i].theta;
		}

		temp_x = temp_x / pf.particles.size();
		temp_y = temp_y / pf.particles.size();
		temp_theta = temp_theta / pf.particles.size();
			
		for (int j = 0; j < observations.size(); j++)
		{
			// Observation measurement transformations
			LandmarkObs observation;
			observation.x = temp_x + (observations[j].x * cos(temp_theta)) - (observations[j].y * sin(temp_theta));
			observation.y = temp_y + (observations[j].x * sin(temp_theta)) + (observations[j].y * cos(temp_theta));
			ob_x.push_back(observation.x);
			ob_y.push_back(observation.y);
		}
			sensor_msgs::PointCloud landmarks_ob;
			landmarks_ob.header.stamp = ros::Time::now();
			landmarks_ob.header.frame_id = "map";
			landmarks_ob.points.resize(observations.size());
			for(int p = 0; p < observations.size(); p++)
			{
				landmarks_ob.points[p].x = ob_x[p];
				landmarks_ob.points[p].y = ob_y[p];
			}
			// cout<<"报错点2"<<endl;
			landmarks_ob_pub.publish(landmarks_ob);
			loop.sleep();
			// cout<<"报错点3"<<endl;


		// Update the weights and resample
		pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		pf.resample();

		//demonstrate the groud truth
		nav_msgs::Odometry odometry_truth;
    	odometry_truth.header.stamp = ros::Time::now();
    	odometry_truth.header.frame_id = "map";
    	odometry_truth.pose.pose.position.x = gt[i].x;
   		odometry_truth.pose.pose.position.y = gt[i].y;
    	odometry_truth.pose.pose.position.z = 0.0;
    	geometry_msgs::Quaternion odom_quat_1 = tf::createQuaternionMsgFromYaw(gt[i].theta);
    	odometry_truth.pose.pose.orientation = odom_quat_1;
		odometry_truth_pub.publish(odometry_truth);
		//延迟一段时间，不然程序运行太快，发布不出来
		loop.sleep();
		
		//demonstrate particle_cloud;
		geometry_msgs::PoseArray cloud_msg;
		cloud_msg.header.stamp = ros::Time::now();
      	cloud_msg.header.frame_id = "map";
      	cloud_msg.poses.resize(pf.particles.size());
		for(int t = 0; t < pf.particles.size(); t++)
		{
			cloud_msg.poses[t].position.x = pf.particles[t].x;
        	cloud_msg.poses[t].position.y = pf.particles[t].y;
        	cloud_msg.poses[t].position.z = 0;
        	tf2::Quaternion q;
        	q.setRPY(0, 0,  pf.particles[t].theta);
        	tf2::convert(q, cloud_msg.poses[t].orientation);
		}
		particlecloud_pub.publish(cloud_msg);
		loop.sleep();
		
		// Calculate and output the average weighted error of the particle filter over all time steps so far.
		vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;
		for (int i = 0; i < num_particles; ++i) {
			if (particles[i].weight > highest_weight) {
				highest_weight = particles[i].weight;
				best_particle = particles[i];
			}
		}

		//demonstrate the best_particle
		nav_msgs::Odometry particle_best;
    	particle_best.header.stamp = ros::Time::now();
    	particle_best.header.frame_id = "map";
    	particle_best.pose.pose.position.x = best_particle.x;
   		particle_best.pose.pose.position.y = best_particle.y;
    	particle_best.pose.pose.position.z = 0.0;
    	geometry_msgs::Quaternion odom_quat_2 = tf::createQuaternionMsgFromYaw(best_particle.theta);
    	particle_best.pose.pose.orientation = odom_quat_2;
		particle_best_pub.publish(particle_best);
		//延迟一段时间，不然程序运行太快，发布不出来
		loop.sleep();

		double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);

		for (int j = 0; j < 3; ++j) {
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}
		
		// Print the cumulative weighted error
		// cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;
		
		// If the error is too high, say so and then exit.
		if (i >= time_steps_before_lock_required) {
			if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error) {
				if (cum_mean_error[0] > max_translation_error) {
					cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else if (cum_mean_error[1] > max_translation_error) {
					cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else {
					cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
				}
				return -1;
			}
		}
	}
	
	// Output the runtime for the filter.
	int stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	// cout << "Runtime (sec): " << runtime << endl;
	
	// Print success if accuracy and runtime are sufficient (and this isn't just the starter code).
	if (runtime < max_runtime && pf.initialized()) {
		cout << "Success! Your particle filter passed!" << endl;
	}
	else if (!pf.initialized()) {
		cout << "This is the starter code. You haven't initialized your filter." << endl;
	}
	else {
		cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
		return -1;
	}
	
	//ros information
	// ros::init(argc, argv, "mcl");
	// ros::NodeHandle n;
	// landmarks_pub = n.advertise<sensor_msgs::PointCloud>("landmarks", 10);

	// sensor_msgs::PointCloud landmarks;
	// ros::Time::init();
	// landmarks.header.stamp = ros::Time::now();
	// landmarks.header.frame_id = "map";
	// landmarks.points.resize(map.landmark_list.size());
	// for(int i = 0; i < map.landmark_list.size(); i++)
	// {
	// 	landmarks.points[i].x = map.landmark_list[i].x_f;
	// 	landmarks.points[i].y = map.landmark_list[i].y_f;
	// }
	// cout<<"没出错"<<endl;
	// landmarks_pub.publish(landmarks);
	// cout<<"没出错"<<endl;

	//ros::Rate loop(200);
    while(ros::ok())
    {        
	// landmarks_pub.publish(landmarks);
	//  odometry_truth_pub.publish(odometry_truth);
       ros::spinOnce();
       loop.sleep();
    }

	return 0;
}


