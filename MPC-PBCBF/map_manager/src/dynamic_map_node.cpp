#include <ros/ros.h>
#include <map_manager/dynamicMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_map_node");
	ros::NodeHandle nh;

	mapManager::dynamicMap m(nh,true);

	ros::spin();

	return 0;
}