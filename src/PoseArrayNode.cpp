/* 
*  
*  Read xx.csv file, load info into vector, then publish topic through 
*  PoseArray.
*
*/


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <geometry_msgs/PoseArray.h>


std::vector<std::vector<double> > tag_poses;

ros::Publisher tag_pub;


std::vector<std::vector<double> > read_poses(std::string fn) {
    std::ifstream pose_fid(fn.c_str());

    std::vector<std::vector<double> > data;
    std::string line;

    while(std::getline(pose_fid, line)) {
        std::vector<double> row;
        std::stringstream iss(line);
        std::string val;

        while (std::getline(iss, val, ',')) {
            double v = std::atof(val.c_str());
            row.push_back(v);
        }
        data.push_back(row);
    }

    return data;
}

void publish_poses(std::vector<std::vector<double> >& poses, ros::Publisher &pub) {
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    // Pod msgs
    for(size_t node_index = 0; node_index < poses.size(); node_index++) {
        geometry_msgs::Pose p;
        p.position.x = poses[node_index][0];
        p.position.y = poses[node_index][1];
        p.position.z = poses[node_index][2];
        p.orientation.w = poses[node_index][3];
        p.orientation.x = poses[node_index][4];
        p.orientation.y = poses[node_index][5];
        p.orientation.z = poses[node_index][6];
        msg.poses.push_back(p);
    }

    pub.publish(msg);
}

void callback(const ros::TimerEvent &e) {
    publish_poses(tag_poses, tag_pub);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_viz");
    ros::NodeHandle n("~");

    tag_pub      = n.advertise<geometry_msgs::PoseArray>("/tag_poses", 10);

    std::string tag_poses_fn;
   

    n.param("tag_poses_fn", tag_poses_fn, std::string("/home/luyao/catkin_ws/src/rviz_textured_quads /resource/tag.csv"));


    // Read tag poses
    tag_poses = read_poses(tag_poses_fn);


    std::cout << "tag poses size " << tag_poses.size()<< " " << tag_poses[0].size() << std::endl;

    ros::Timer timer = n.createTimer(ros::Duration(1), callback);

    ros::spin(); 

    return 0;
}
