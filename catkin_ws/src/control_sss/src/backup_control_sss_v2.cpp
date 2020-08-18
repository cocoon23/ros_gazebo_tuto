//2020-06-01
//program: control sss model in gazebo and save pointcloud and image
//recieve from stereo camera

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

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
rosbag::Bag bag;

void image_left_sub_callback(const sensor_msgs::Image& img_left)
{
	bag.write("/sss/left/image_raw", ros::Time::now(), img_left);
	std::cout << "bag1" <<std::endl;
}

void image_right_sub_callback(const sensor_msgs::Image& img_right)
{
	bag.write("/sss/right/image_raw", ros::Time::now(), img_right);
        std::cout << "bag2" <<std::endl;
}


void pt_sub_callback(const sensor_msgs::PointCloud2& msg_org)
{
	bag.write("/sss/points2", ros::Time::now(), msg_org);
        std::cout << "bag3" <<std::endl;

	sensor_msgs::PointCloud msg;
        sensor_msgs::convertPointCloud2ToPointCloud(msg_org, msg);
        printf("%f\n", msg.points[0].x);

        printf("%d\n", msg.points.size());

        int point_num = msg.points.size();
	std::ofstream point_result("/mnt/hgfs/shared_folder/point_result.txt");
        //for (int i = 0; i < point_num; i++){
        //      printf("\t(%f, %f, %f)\n", msg.points[i], msg.points[i], msg.points[i]);
        //}

        double left_camera_y_cdn = 0.0;
        double right_camera_y_cdn = -0.7;
        double z_filter = 0.01;

        double pc_sum_x = 0.0;
        double pc_sum_y = 0.0;
        double pc_sum_z = 0.0;

        double pc_cof_x = 0.0;
        double pc_cof_y = 0.0;
        double pc_cof_z = 0.0;

        for (int i = 0; i < point_num; i++){
                if (msg.points[i].z <= z_filter){
                        continue;
                }

                pc_sum_x += msg.points[i].x;
                pc_sum_y += msg.points[i].y;
                pc_sum_z += msg.points[i].z;
        }

        pc_cof_x = pc_sum_x / point_num;
        pc_cof_y = pc_sum_y / point_num;
        pc_cof_z = pc_sum_z / point_num;


        int object_position_case;
        if (pc_cof_y < left_camera_y_cdn){
                object_position_case = 0;
	}
        else if (pc_cof_y >= left_camera_y_cdn && pc_cof_y < right_camera_y_cdn){
                object_position_case = 1;
        }
        else{
                object_position_case = 2;
        }


        // Find the point that has the widest angle relative to the base (left) camera's forward vector (0, 0, 1)
        double forward_vector_x = 0.0;
        double forward_vector_y = 0.0;
        double forward_vector_z = 1.0;
        double mag_forward_vector = sqrt(pow(forward_vector_x, 2) + pow(forward_vector_y, 2) + pow(forward_vector_z, 2));

        double widest_angle = 0.0;
        int widest_index = -1;
        if (object_position_case == 0 || object_position_case == 2){
                double curr_x = 0.0;
                double curr_y = 0.0;
                double curr_z = 0.0;

                double dot_product = 0.0;
                double mag_curr_point = 0.0;
                double angle = 0.0;
                for (int i = 0; i < point_num; i++){
                        curr_x = msg.points[i].x;
                        curr_y = msg.points[i].y;
                        curr_z = msg.points[i].z;

                        if (curr_z <= z_filter){
                                continue;
                        }


                        dot_product = forward_vector_x * curr_x + forward_vector_y * curr_y + forward_vector_z * curr_z;

                        mag_curr_point = sqrt(pow(curr_x, 2) + pow(curr_y, 2) + pow(curr_z, 2));

                        angle = std::acos(dot_product/(mag_forward_vector * mag_curr_point));

                        if (std::abs(angle) > widest_angle){
                                widest_index = i;
                        }
                }

                printf("Case: %d, (%f, %f, %f)\n", object_position_case, msg.points[widest_index].x, msg.points[widest_index].y, msg.points[widest_index].z);
		point_result << msg.points[widest_index].x<< " " << msg.points[widest_index].y << " " << msg.points[widest_index].z<<std::endl;
        }

        else if (object_position_case == 1){
                printf("Case: %d, (%f, %f, %f)\n", object_position_case, pc_cof_x, pc_cof_y, pc_cof_z);
		point_result << pc_cof_x<< " " << pc_cof_y << " " << pc_cof_z<<std::endl;
        }
	point_result.close();

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
	ros::Rate loop_rate(1);
	
	//if(!nh.getParam("/file_name", file_name)){
	//	nh.param<std::string>("/file_name", file_name, "/mnt/hgfs/shared_folder/result.txt");
	//}
        ros::Subscriber sub_image_left = nh.subscribe("/sss/left/image_raw", 1000,image_left_sub_callback);
	ros::Subscriber sub_image_right = nh.subscribe("/sss/right/image_raw", 1000, image_right_sub_callback);
	ros::Subscriber sub_pt = nh.subscribe("/sss/points2", 1000, pt_sub_callback);
	
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
				
			        save_filename = std::to_string(ms.pose.position.x)+"_" + std::to_string(ms.pose.position.y)+"_" +std::to_string(ms.pose.position.z)+"_" + vangle[0]+"_"+vangle[1] +".bag";
				                               
                                //message_filters::Subscriber<sensor_msgs::Image> image_sub_right(nh, "/sss/right/image_raw", 1);
                                //message_filters::Subscriber<sensor_msgs::Image> image_sub_left(nh, "/sss/left/image_raw", 1);

                                //message_filters::Subscriber<sensor_msgs::PointCloud2> pt_sub(nh, "sss/points2", 1);
				//ros::Subscriber sub_image_left = nh.subscribe("/sss/left/image_raw", 1000,image_left_sub_callback);
				//ros::Subscriber sub_image_right = nh.subscribe("/sss/right/image_raw", 1000, image_right_sub_callback);
				//ros::Subscriber sub_pt = nh.subscribe("/sss/points2", 1000, pt_sub_callback);
                                //image_sub_left.registerCallback(image_left_sub_callback);
                                //image_sub_right.registerCallback(image_right_sub_callback);
                                //pt_sub.registerCallback(pt_sub_callback);

				
				
				bag.open(save_filename, rosbag::bagmode::Write);
				ros::spinOnce();
				loop_rate.sleep();
				bag.close();
				line++;

			}
			i++;
		}
		infile.close();

				
	}


	

}


