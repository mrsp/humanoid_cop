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


#include <iostream>
#include "humanoid_cop/humanoid_cop.h"


humanoid_cop::~humanoid_cop() {
	if (is_connected_)
		disconnect();
}

void humanoid_cop::disconnect() {
	if (!is_connected_)
		return;
	
	is_connected_ = false;
}



humanoid_cop::humanoid_cop()
{
	FT_inc = false;
	copl = Vector3d::Zero();
	copr = Vector3d::Zero();
}
void humanoid_cop::run() {
	static ros::Rate rate(2.5*FT_freq);  //ROS Node Loop Rate
	while (ros::ok()) {
		if(FT_inc){
			computeCOP();
			publishCOP();
			FT_inc = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
}

bool humanoid_cop::connect(const ros::NodeHandle nh) {
	// Initialize ROS nodes
	n = nh;
	// Load ROS Parameters
	loadparams();
	//Subscribe/Publish ROS Topics/Services
	subscribe();
	advertise();

	is_connected_ = true;


	return true;
}



bool humanoid_cop::connected() {
	return is_connected_;
}

void humanoid_cop::subscribe()
{
	subscribeToFT();
	ros::Duration(1.0).sleep();
}



void humanoid_cop::computeCOP() {
	// Computation of the CoP in the Local Coordinate Frame of the Foot
	copl.setZero();
	copl(0) = -lFT_msg.wrench.torque.y/lFT_msg.wrench.force.z;
	copl(1) = -lFT_msg.wrench.torque.x/lFT_msg.wrench.force.z;
	copl(2) = 0.0;
	copr.setZero();
	copr(0) = -rFT_msg.wrench.torque.y/rFT_msg.wrench.force.z;
	copr(1) =  -rFT_msg.wrench.torque.x/rFT_msg.wrench.force.z;
	copr(2) =  0.0;
}




void humanoid_cop::loadparams() {

	ros::NodeHandle n_p("~");
	n_p.param<double>("FT_topic_freq",FT_freq,500.0);
	n_p.param<std::string>("lFT_topic",lFT_topic,"lFT");
	n_p.param<std::string>("rFT_topic",rFT_topic,"rFT");
	n_p.param<std::string>("lcop_topic",lcop_topic,"lcop");
	n_p.param<std::string>("rcop_topic",rcop_topic,"rcop");
}



void humanoid_cop::subscribeToFT()
{
	//Left Foot
	lFT_sub = n.subscribe(lFT_topic,1000,&humanoid_cop::lFTCb,this);
	//Right Foot
	rFT_sub = n.subscribe(rFT_topic,1000,&humanoid_cop::rFTCb,this);
}

void humanoid_cop::lFTCb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	lFT_msg = *msg;
	FT_inc = true;
}
void humanoid_cop::rFTCb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rFT_msg = *msg;
}


void humanoid_cop::advertise() {

	COPL_pub = n.advertise<geometry_msgs::PointStamped>(lcop_topic,1000);
	COPR_pub = n.advertise<geometry_msgs::PointStamped>(rcop_topic,1000);

}
void humanoid_cop::publishCOP() {
	COP_msg.point.x = copl(0);
	COP_msg.point.y = copl(1);
	COP_msg.point.z = copl(2);
	COP_msg.header.stamp = ros::Time::now();
	COPL_pub.publish(COP_msg);
	COP_msg.point.x = copr(0);
	COP_msg.point.y = copr(1);
	COP_msg.point.z = copr(2);
	COP_msg.header.stamp = ros::Time::now();
	COPR_pub.publish(COP_msg);
}