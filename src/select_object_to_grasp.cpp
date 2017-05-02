#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath> 

#include "semantic_mapping/PanTilt.h"

ros::Publisher gaze_pub;

using namespace std;

std::string all_classes ("person bicycle car motorcycle airplane bus train truck boat trafficlight firehydrant stopsign parkingmeter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee skis snowboard sportsball kite baseballbat baseballglove skateboard surfboard tennisracket bottle wineglass cup fork knife spoon bowl banana apple sandwich orange broccoli carrot hotdog pizza donut cake chair couch pottedplant bed diningtable toilet tv laptop mouse remote keyboard cellphone microwave oven toaster sink refrigerator book clock vase scissors teddybear hairdrier toothbrush handle");



void look_around_callback()
{
		sleep(1);
		ROS_INFO(" Looking around to create octomap");
		semantic_mapping::PanTilt gaze_l, gaze_c, gaze_r;
		// Right	
		gaze_l.pan_angle.data=-0.8;
		gaze_l.tilt_angle.data=0.5;
		gaze_pub.publish(gaze_l);
		sleep(3);
		// Right- down
		gaze_l.pan_angle.data=-0.8;
		gaze_l.tilt_angle.data=0.7;
		gaze_pub.publish(gaze_l);
		sleep(3);
		// Left	- DOWN
		gaze_r.pan_angle.data=0.3;
		gaze_r.tilt_angle.data=0.7;
		gaze_pub.publish(gaze_r);
		sleep(3);
		// Left		
		gaze_r.pan_angle.data=0.3;
		gaze_r.tilt_angle.data=0.5;
		gaze_pub.publish(gaze_r);
		sleep(3);
		// Center		
		gaze_c.pan_angle.data=0.0;
		gaze_c.tilt_angle.data=0.5;
		gaze_pub.publish(gaze_c);
}


int main(int argc, char *argv[])
{
    // Wait for agile_grasp launcher to print parameters
    //sleep(1);

    ros::init(argc, argv, "select_object_to_grasp");
    ros::NodeHandle nh("~");

		// Publish gaze messages to see environment
		gaze_pub = nh.advertise<semantic_mapping::PanTilt>("/pan_tilt_angle", 5);
		look_around_callback();


    string str_quit = "q";

    string className = "";
    cout << "Please enter name of class to gasp (or 'q' to quit):\n>";

    while(1)
    {
      // Get className
      getline(cin, className);

      // Check lists to see what clustering method to use
	    std::size_t found = all_classes.find(className);
   
      if (str_quit.compare(className) == 0)
			{
				break;
			}else if (found!=std::string::npos)
			{
        cout << "You entered: " << className << endl << endl;
      
        // Build TopicName
        string topicName;
        topicName = "/pcl_registered/"+ className;

        // Create parameter for launch file
        nh.param<string>("topicName", topicName);
        ros::param::set("topicName", topicName);

        ROS_INFO("topicName = %s", topicName.c_str());
        ROS_INFO("className = %s", className.c_str());

				string system_str;
        system_str = "roslaunch semantic_mapping select_object_grasp.launch topicName:=" + topicName + " className:=" + className;
        system(system_str.c_str());

 				cout << "\nPlease enter name of class to gasp (or 'q' to quit):\n>";

      }
      else {
				cout << "\nClass does not exist. Please enter name of class to gasp again (or 'q' to quit):\n>";
			}
		}

    return 0;
}
