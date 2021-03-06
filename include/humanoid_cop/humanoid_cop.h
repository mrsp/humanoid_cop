/*
 * humanoid_state_estimation - a complete state estimation scheme for humanoid robots
 *
 * Copyright 2017-2018 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HUMANOID_COP_H
#define HUMANOID_COP_H

// ROS Headers
#include <ros/ros.h>


#include <eigen3/Eigen/Dense>
// ROS Messages
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>



using namespace Eigen;
using namespace std;

class humanoid_cop{
private:
	// ROS Standard Variables
	ros::NodeHandle n;

	ros::Publisher COPL_pub, COPR_pub;
    ros::Subscriber lFT_sub, rFT_sub;
	
	double  FT_freq;
	bool FT_inc;
	std::string lcop_topic, rcop_topic;
	//ROS Messages
	

	geometry_msgs::WrenchStamped lFT_msg,  rFT_msg;
   
	geometry_msgs::PointStamped COP_msg;

	// Helper
	bool is_connected_;



	Vector3d copl, copr;




	/** Real odometry Data **/
    std::string lFT_topic,rFT_topic;
	
	//Odometry, from supportleg to inertial, transformation from support leg to other leg
	 void subscribeToFT();

	 void lFTCb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void rFTCb(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	 void computeCOP();



	void publishCOP();
	// Advertise to ROS Topics
	void advertise();
	void subscribe();

public:


	// Constructor/Destructor
	humanoid_cop();

	~humanoid_cop();

	// Connect/Disconnet to ALProxies
	bool connect(const ros::NodeHandle nh);

	void disconnect();



	// Parameter Server
	void loadparams();

	void run();

	bool connected();


};

#endif // HUMANOID_COP_H
