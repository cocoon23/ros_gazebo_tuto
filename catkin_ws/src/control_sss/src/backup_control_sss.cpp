//2020-06-01
//program: control sss model in gazebo and save pointcloud and image
//recieve from stereo camera

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

std::string save_filename;
/*
sensor_msgs::Image image_sub_callback(const sensor_msgs::ImageConstPtr& image)
{
	
}
*/
/*
void callback(const sensor_msgs::ImageConstPtr& im_left, const sensor_msgs::ImageConstPtr& im_right, const sensor_msgs::PointCloud2ConstPtr& pt)
{
	std::cout<<"hi"<<std::endl;
}
*/

void callback(const sensor_msgs::ImageConstPtr& im_left, const sensor_msgs::ImageConstPtr& im_right)
{
        std::cout<<"hi"<<std::endl;
}


std::vector<std::string> split(std::string targetStr, std::string token)
{
    if(token.length() == 0 || targetStr.find(token) == std::string::npos)
    return std::vector<std::string>({targetStr});
 
    // return var
    std::vector<std::string> ret;
 
    int findOffset  = 0;
    int splitOffset = 0;
    while ((splitOffset = targetStr.find(token, findOffset)) != std::string::npos)
    {
         ret.push_back(targetStr.substr(findOffset, splitOffset - findOffset));
         findOffset = splitOffset + token.length();
    }
    ret.push_back(targetStr.substr(findOffset, targetStr.length() - findOffset));
    
    return ret;


//출처: https://5kyc1ad.tistory.com/289 [Re: 제로부터 시작하는 블로그 생활]


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_sss");
	ros::NodeHandle nh;
	gazebo_msgs::ModelState ms;
	gazebo_msgs::SetModelConfiguration mc;
	std::string temp;
	//set start position;
	ms.model_name = "sss";
	mc.request.model_name = "sss";
	mc.request.urdf_param_name = "robot_description";
	mc.request.joint_names = {"left_camera_joint","right_camera_joint"};
	std::vector<std::string> vstart_pos;
	std::ifstream fstart_pos("start_position.txt");
	rosbag::Bag bag;

	if(!fstart_pos.fail())
	{
		std::getline(fstart_pos, temp);
		vstart_pos = split(temp, " ");
		ms.pose.position.x = std::stof(vstart_pos[0]);
		ms.pose.position.y = std::stof(vstart_pos[1]);
		ms.pose.position.z = std::stof(vstart_pos[2]);
	}
	else std::cout<<"cannot open start_postion.txt" <<std::endl;
	

	std::string file_name;
	std::string angle;
	std::vector<std::string> vangle;
	int line=0;

	//ros::Publisher camera_rotate = nh.adverise<gasedo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration", 1000);
	ros::Rate loop_rate(10);
	
	//if(!nh.getParam("/file_name", file_name)){
	//	nh.param<std::string>("/file_name", file_name, "/mnt/hgfs/shared_folder/result.txt");
	//}
	
	while(ros::ok())
	{
		int i = 0;
		std::ifstream infile;
		infile.open("/mnt/hgfs/shared_folder/result.txt");
		//infile.open("/home/meungsuklee/catkin_ws/src/control_sss/src/result.txt");
		if(!infile.is_open()) 
		{
			std::cout<<"cannot open result.txt"<<std::endl;
		}
		while(!infile.eof())
		{
			//infile.getline(angle, 100);
			std::getline(infile, angle);
			//std::cout<<i<<std::endl;
			//std::cout<<angle<<std::endl;
			if(i == line)
			{
				std::cout<<i << line << std::endl;
			}

			if(i == line)
			{
				//ros::Subscriber sub = nh.subscribe("bag", 1000, save_callback);
				std::vector<std::string> topics;
				//std::cout<<"result angle is "<<angle<<std::endl;
				vangle = split(angle, " ");
				std::cout << "result angle is: "<<vangle[0] << "," <<vangle[1] << std::endl;
				mc.request.joint_positions = {std::stof(vangle[0]), std::stof(vangle[1])};
				ros::service::call("gazebo/set_model_configuration", mc);
				
			        save_filename = std::to_string(ms.pose.position.x) + std::to_string(ms.pose.position.y) +std::to_string(ms.pose.position.z) + vangle[0] +vangle[1] +".bag";
				                               
			       	sensor_msgs::Image im_left, im_right;
                                sensor_msgs::PointCloud2 pt;
                                message_filters::Subscriber<sensor_msgs::Image> image_sub_right(nh, "/sss/right/image_raw", 1);
                                message_filters::Subscriber<sensor_msgs::Image> image_sub_left(nh, "/sss/left/image_raw", 1);

                                message_filters::Subscriber<sensor_msgs::PointCloud2> pt_sub(nh, "sss/points2", 1);
                                //im_left = image_sub_left.registerCallback(image_sub_callback);
                                //im_right = image_sub_right.registerCallback(image_sub_callback);
                                //pt = pt_sub.registereCallback(pt_sub_callback);
                                //i

				//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub_right, image_sub_left, pt_sub, 10);
				
				//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub_right, image_sub_left, 10);
				//
				typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
				typedef message_filters::Synchronizer<MySyncPolicy> MySynchronizer;
				boost::shared_ptr<MySynchronizer> sync;
				sync.reset(new MySynchronizer(MySyncPolicy(10), image_sub_left, image_sub_right));
				//message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_left, image_sub_right);
				sync->registerCallback(boost::bind(&callback, _1, _2));
				
				//bag.open(save_filename, rosabg::bagmod::Read);
				//camera_rotate.publish(mc);
				//ros::spinOnce();
				//loop_rate.sleep();
				ros::spinOnce();
				loop_rate.sleep();
				line++;

			}
			i++;
		}
		infile.close();

				
	}


	

}


