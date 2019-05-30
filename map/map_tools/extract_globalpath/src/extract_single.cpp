#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>

namespace TEST{
class test_nav{
    private:
        geometry_msgs::PoseStamped pose;
        nav_msgs::Path global_path;

        ros::Subscriber sub_pose;
        ros::Publisher pub_path;

        std::string path_file;
        std::vector<int> count;

        double weight_data;
        double weight_smooth;
        double tolerance;

        void pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

        void insert_pose(const geometry_msgs::PoseStamped pose);

        void smooth_path(nav_msgs::Path &path, double weight_data, double weight_smooth, double tolerance);

        double distance2points(const geometry_msgs::PoseStamped p1,const geometry_msgs::PoseStamped p2);

        void write_back();

    public:
    test_nav(){
        ros::NodeHandle nh("~");
        nh.param<std::string>("path_file",path_file,"None");
        std::cout << path_file << std::endl;

        nh.param<double>("weight_data",weight_data,0.47);
        nh.param<double>("weight_smooth",weight_smooth,0.2);
        nh.param<double>("tolerance",tolerance,0.2);
    }
    
    void init();
};
}



void TEST::test_nav::pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    geometry_msgs::PoseStamped temp;
    temp.pose = msg->pose.pose;
    insert_pose(temp);
    smooth_path(global_path,weight_data,weight_smooth,tolerance);
    pub_path.publish(global_path);
    write_back();
}

void TEST::test_nav::insert_pose(const geometry_msgs::PoseStamped pose){
    if(global_path.poses.empty()){
        global_path.poses.push_back(pose);
        return;
    }
    int size = global_path.poses.size();
    geometry_msgs::PoseStamped start = global_path.poses[size-1];
    geometry_msgs::PoseStamped end = pose;
    if(distance2points(start,end) <= 1.0){
        global_path.poses.push_back(pose);
        return;
    }
    double lenx = end.pose.position.x - start.pose.position.x;
    double leny = end.pose.position.y - start.pose.position.y;
    int cnt;
    if(std::abs(lenx)>std::abs(leny)){
        cnt = int(std::abs(lenx))*2;
    }else{
        cnt = int(std::abs(leny))*2;
    }
    double dx = lenx/cnt;
    double dy = leny/cnt;
    std::cout << "--------------------------" << std::endl;
    std::cout << "lenx: " << lenx << std::endl;
    std::cout << "leny: " << leny << std::endl;
    std::cout << "cnt: " << cnt << std::endl;
    std::cout << "dx: " << dx << std::endl;
    std::cout << "dy: " << dy << std::endl;
    std::cout << "--------------------------" << std::endl;

    for(int i=1;i<=cnt;i++){
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = start.pose.position.x + i * dx;
        temp.pose.position.y = start.pose.position.y + i * dy;
        global_path.poses.push_back(temp);
    }
    std::cout << "path size: " << global_path.poses.size() << std::endl;
}

double TEST::test_nav::distance2points(const geometry_msgs::PoseStamped p1,const geometry_msgs::PoseStamped p2){
    return std::sqrt(std::pow(p2.pose.position.x-p1.pose.position.x,2)+std::pow(p2.pose.position.y-p1.pose.position.y,2));
}

void TEST::test_nav::smooth_path(nav_msgs::Path &path, double weight_data, double weight_smooth, double tolerance){
    int size = global_path.poses.size();
    count.resize(size,0);

	if (path.poses.size() <= 2)
	{
		//cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
		return;
	}
    std::cout << "Smooth the path." << std::endl;
	const nav_msgs::Path &path_in = path;
	nav_msgs::Path smoothPath_out = path_in;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
			//			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
			//				continue;
            if(count[i] >= 10) continue;
			xtemp = smoothPath_out.poses[i].pose.position.x;
			ytemp = smoothPath_out.poses[i].pose.position.y;

			smoothPath_out.poses[i].pose.position.x += weight_data * (path_in.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
			smoothPath_out.poses[i].pose.position.y += weight_data * (path_in.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

			smoothPath_out.poses[i].pose.position.x += weight_smooth * (smoothPath_out.poses[i - 1].pose.position.x + smoothPath_out.poses[i + 1].pose.position.x - (2.0 * smoothPath_out.poses[i].pose.position.x));
			smoothPath_out.poses[i].pose.position.y += weight_smooth * (smoothPath_out.poses[i - 1].pose.position.y + smoothPath_out.poses[i + 1].pose.position.y - (2.0 * smoothPath_out.poses[i].pose.position.y));

			change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
			change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
            count[i]++;
		}
		nIterations++;
	}

	path = smoothPath_out;
}

void TEST::test_nav::init(){
    ros::NodeHandle nh;
    global_path.header.frame_id="map";
    sub_pose = nh.subscribe("/initialpose", 1, &TEST::test_nav::pose_cb, this);
    pub_path = nh.advertise<nav_msgs::Path>("/global_path",1);
}

void TEST::test_nav::write_back(){
    std::ofstream f;
    f.open(path_file.c_str(),std::ios::out);
    if(!f.is_open()){
        ROS_ERROR_STREAM("Can not open the file to save path, exit.");
        return;
    }
    int size = global_path.poses.size();
    for(int i=0;i<size;i++){
        pose = global_path.poses[i];
        f << pose.pose.position.x << ","
          << pose.pose.position.y << ","
          << pose.pose.position.z << ","
          << pose.pose.orientation.x << ","
          << pose.pose.orientation.y << ","
          << pose.pose.orientation.z << ","
          << pose.pose.orientation.w << ","
          << std::endl;
    }
    f.close();
}

int main(int argc,char** argv){
    ros::init(argc,argv,"test_nav_msgs");
    TEST::test_nav app;
    app.init();
    ros::spin();
    return 0;
}