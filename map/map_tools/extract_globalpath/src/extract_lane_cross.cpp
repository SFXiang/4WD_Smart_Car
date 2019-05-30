#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <sstream>

#include <path_msgs/Lane.h>
#include <path_msgs/Cross.h>
#include <path_msgs/choose.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <algorithm>

namespace WHOLE_PATH{
class extract_path{
private:
    ros::Subscriber sub_pose;
    ros::Subscriber sub_type;
    ros::Publisher pub_path;
    ros::Publisher pub_markerArray;
    ros::Publisher pub_texts;
    int cnt_text;

    nav_msgs::Path current_path;
    // path_msgs::Lane *lane_list;
    // path_msgs::Cross *cross_list;
    std::vector<path_msgs::Lane> lane_list;
    std::vector<path_msgs::Cross> cross_list;

    std::string path_prefix;
    bool is_begin;
    int path_type;

    double weight_data;
    double weight_smooth;
    double tolerance;

    visualization_msgs::MarkerArray lines,MapPoints;
    visualization_msgs::MarkerArray texts;

    void pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void insert_pose(const geometry_msgs::PoseStamped pose);

    void smooth_path(double weight_data, double weight_smooth, double tolerance);

    void type_cb(const path_msgs::choose type);

    double distance2points(const geometry_msgs::PoseStamped p1,const geometry_msgs::PoseStamped p2);

    void show();
    void show_tests(std::string label, int id, std::vector<int> &pre_id, std::vector<int> &next_id, int length, bool reverse, geometry_msgs::Pose pose);

    void write_back();
    void write_func(const std::string path_type, const int id);

public:
    extract_path(){
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("path_prefix",path_prefix,"None");
        std::cout << "Result will be saved at " << path_prefix << std::endl;

        pnh.param<double>("weight_data",weight_data,0.47);
        pnh.param<double>("weight_smooth",weight_smooth,0.2);
        pnh.param<double>("tolerance",tolerance,0.2); 

        // lane_list = new path_msgs::Lane[100];
        // cross_list = new path_msgs::Cross[100];
        lane_list.resize(101);
        cross_list.resize(101);
        is_begin = true;
        cnt_text = 0;
    }

    void init();    
};

}

void WHOLE_PATH::extract_path::init(){
    ros::NodeHandle nh;
    current_path.header.frame_id="map";
    sub_pose = nh.subscribe("/initialpose", 1, &WHOLE_PATH::extract_path::pose_cb, this);
    sub_type = nh.subscribe("/switch",1,&WHOLE_PATH::extract_path::type_cb, this);
    pub_path = nh.advertise<nav_msgs::Path>("/current_path",1);
    pub_markerArray = nh.advertise<visualization_msgs::MarkerArray>("/path/history",10);
    pub_texts = nh.advertise<visualization_msgs::MarkerArray>("/path/label",1);
}

void WHOLE_PATH::extract_path::pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    if(is_begin){
        current_path.poses.clear();
        is_begin = false;
    }
    geometry_msgs::PoseStamped temp;
    temp.pose = msg->pose.pose;
    insert_pose(temp);
    smooth_path(weight_data,weight_smooth,tolerance);
    pub_path.publish(current_path);
}

void WHOLE_PATH::extract_path::insert_pose(const geometry_msgs::PoseStamped pose){
    if(current_path.poses.empty()){
        current_path.poses.push_back(pose);
        return;
    }
    int size = current_path.poses.size();
    geometry_msgs::PoseStamped start = current_path.poses[size-1];
    geometry_msgs::PoseStamped end = pose;
    if(distance2points(start,end) <= 1.0){
        current_path.poses.push_back(pose);
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
    // std::cout << "--------------------------" << std::endl;
    // std::cout << "lenx: " << lenx << std::endl;
    // std::cout << "leny: " << leny << std::endl;
    // std::cout << "cnt: " << cnt << std::endl;
    // std::cout << "dx: " << dx << std::endl;
    // std::cout << "dy: " << dy << std::endl;
    // std::cout << "--------------------------" << std::endl;

    for(int i=1;i<=cnt;i++){
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = start.pose.position.x + i * dx;
        temp.pose.position.y = start.pose.position.y + i * dy;
        current_path.poses.push_back(temp);
    }
    std::cout << "path size: " << current_path.poses.size() << std::endl;
}

void WHOLE_PATH::extract_path::smooth_path(double weight_data, double weight_smooth, double tolerance){
    int size = current_path.poses.size();
    int count[size+1];
    for(size_t i = 0;i <= size; i++) count[i] = 0;

	if (current_path.poses.size() <= 2)
	{
		//cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
		return;
	}
    std::cout << "Smooth the path." << std::endl;
	const nav_msgs::Path &path_in = current_path;
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
            if(count[i] >= 20) continue;
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
	current_path = smoothPath_out;
}

double WHOLE_PATH::extract_path::distance2points(const geometry_msgs::PoseStamped p1,const geometry_msgs::PoseStamped p2){
    return std::sqrt(std::pow(p2.pose.position.x-p1.pose.position.x,2)+std::pow(p2.pose.position.y-p1.pose.position.y,2));
}

void WHOLE_PATH::extract_path::type_cb(const path_msgs::choose type){
    if(current_path.poses.empty()){
        ROS_WARN_STREAM("Current path unset! Please draw current path first.");
        return;
    }
    if(strcmp("writeback",type.type.c_str())==0){
        write_back();
        return;
    }
    else if(strcmp("lane",type.type.c_str()) == 0){
        if(lane_list[type.id].path.poses.size() > 0){
            lane_list[type.id].pre_id.clear();
            lane_list[type.id].next_id.clear();
            lane_list[type.id].path.poses.clear();
            texts.markers.pop_back();
            cnt_text--;
        }
        path_type = 1;

        // calculate the length of current_path
        double path_length = 0.0;
        for(int i=0;i<current_path.poses.size()-1;i++){
            path_length += distance2points(current_path.poses[i],current_path.poses[i+1]);
        }

        lane_list[type.id].id = type.id;
        lane_list[type.id].reverse = type.reverse;
        lane_list[type.id].length = path_length;
        for(int i = 0; i < type.pre_id.size(); i++){
            lane_list[type.id].pre_id.push_back(type.pre_id[i]);
        }
        for(int i = 0; i < type.next_id.size(); i++){
            lane_list[type.id].next_id.push_back(type.next_id[i]);
        }

        lane_list[type.id].path = current_path;
        int size = lane_list[type.id].path.poses.size();
        std::cout << "Insert lane id: " << type.id << " with " << size << " points" << std::endl;

        show_tests("lane", lane_list[type.id].id, lane_list[type.id].pre_id, lane_list[type.id].next_id, lane_list[type.id].length, lane_list[type.id].reverse, lane_list[type.id].path.poses[int(size/2)].pose);
    }
    else if(strcmp("cross",type.type.c_str()) == 0){
        if(cross_list[type.id].path.poses.size() > 0){
            cross_list[type.id].pre_id.clear();
            cross_list[type.id].next_id.clear();
            cross_list[type.id].path.poses.clear();
            texts.markers.pop_back();
            cnt_text--;
        }
        path_type = 0;

        // calculate the length of current_path
        double path_length = 0.0;
        for(int i=0;i<current_path.poses.size()-1;i++){
            path_length += distance2points(current_path.poses[i],current_path.poses[i+1]);
        }

        cross_list[type.id].id = type.id;
        cross_list[type.id].reverse = type.reverse;
        cross_list[type.id].length = path_length;
        for(int i = 0; i < type.pre_id.size(); i++){
            cross_list[type.id].pre_id.push_back(type.pre_id[i]);
        }
        for(int i = 0; i < type.next_id.size(); i++){
            cross_list[type.id].next_id.push_back(type.next_id[i]);
        }

        cross_list[type.id].path = current_path;
        int size = cross_list[type.id].path.poses.size();
        std::cout << "Insert cross id: " << type.id << " with " << size << " points" << std::endl;
        show_tests("cross", cross_list[type.id].id, cross_list[type.id].pre_id, cross_list[type.id].next_id, cross_list[type.id].length, cross_list[type.id].reverse, cross_list[type.id].path.poses[int(size/2)].pose);
    }
    else{
        ROS_WARN_STREAM("Wrong switch message, please check.");
        return;
    }

    is_begin = true;
    show();
}

void WHOLE_PATH::extract_path::show(){
    static int cnt = 1;
    for(int j=0;j<current_path.poses.size()-1;j++){
        visualization_msgs::Marker line_strip,points;
        line_strip.header.frame_id = points.header.frame_id = "map";
        line_strip.header.stamp = points.header.stamp = ros::Time::now();
        // line_strip.ns = "WHOLE_PATH";
        line_strip.action = visualization_msgs::Marker::ADD;

        line_strip.id = cnt;  // the id is unique. or you can use 'lifetime' to flush out the older id with newer one
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.2;  // meters
        if(path_type == 1){
            line_strip.color.b = 1.0;  // in the range of [0, 1]
            line_strip.color.a = 1.0;  // Don't forget to set the alpha! default:0 means transparent
        }else{
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;   
        }
        
        points.id = cnt;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 1;  
        points.scale.y = 1;
        points.color.r = 1.0f;
        points.color.a = 1.0;

        geometry_msgs::Point p,q;
        p.x = current_path.poses[j].pose.position.x;
        p.y = current_path.poses[j].pose.position.y;
        // p.z = current_path.poses[j].pose.position.z;
        q.x = current_path.poses[j+1].pose.position.x;
        q.y = current_path.poses[j+1].pose.position.y;
        // q.z = current_path.poses[j+1].pose.position.z;
        
        line_strip.points.push_back(p);
        line_strip.points.push_back(q);
        points.points.push_back(p);
        points.points.push_back(q);

        lines.markers.push_back(line_strip);
        MapPoints.markers.push_back(points);
        cnt++;
    }

    pub_markerArray.publish(lines);
    // pub_markerArray.publish(MapPoints);
}

void WHOLE_PATH::extract_path::show_tests(std::string label, int id, std::vector<int> &pre_id, std::vector<int> &next_id, int length, bool reverse, geometry_msgs::Pose pose){
    visualization_msgs::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = ros::Time::now();
    text.action = visualization_msgs::Marker::ADD;
    text.pose = pose;

    text.id = cnt_text++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    text.scale.z = 2.0;
    text.color.g = 1;  // color range: [0,1]
    text.color.b = 0;
    text.color.r = 0;
    text.color.a = 1;

    std::stringstream str;
    str << label << " " << id << "\n";
    str << "length: " << length << "\n";
    str << "pre_id: ";
    for(int i=0; i<pre_id.size(); i++){
        str << pre_id[i] << " ";
    }
    str << "\n";

    str << "next_id: ";
    for(int i=0; i<next_id.size(); i++){
        str << next_id[i] << " ";
    }
    str << "\n";

    str << "reverse: ";
    if(reverse) str << "True";
    else str << "False";

    text.text = str.str();
    texts.markers.push_back(text);
    pub_texts.publish(texts);
}

void WHOLE_PATH::extract_path::write_back(){
    // int lane_size,cross_size;
    // for(lane_size = 0;!lane_list[lane_size].path.poses.empty();lane_size++);
    // for(cross_size = 0;!cross_list[cross_size].path.poses.empty();cross_size++);
    std::cout << "********* The Lane list is as followed *************" << std::endl;
    for(int i=0;i<101;i++){
        if(lane_list[i].path.poses.empty()) continue;
        // pre
        std::cout << "lane id: " << i << std::endl;
        int before_size = lane_list[i].pre_id.size();
        std::cout << "(pre)before size: " << before_size << std::endl;
        for (int j=0;j<before_size;j++) std::cout << lane_list[i].pre_id[j] << " ";
        std::cout << std::endl;

        // std::vector<int>::iterator it1 = std::unique(lane_list[i].pre_id.begin(),lane_list[i].pre_id.end());
        // lane_list[i].pre_id.erase(it1,lane_list[i].pre_id.end());
        // std::sort(lane_list[i].pre_id.begin(),lane_list[i].pre_id.end());

        // int after_size = lane_list[i].pre_id.size();
        // std::cout << "(pre)after  size: " << after_size << std::endl;
        // for (int j=0;j<after_size;j++) std::cout << lane_list[i].pre_id[j] << " ";
        // std::cout << std::endl << std::endl;
        
        // next
        int before_size1 = lane_list[i].next_id.size();
        std::cout << "(next)before size: " << before_size1 << std::endl;
        for (int j=0;j<before_size1;j++) std::cout << lane_list[i].next_id[j] << " ";
        std::cout << std::endl;

        // std::vector<int>::iterator it2 = std::unique(lane_list[i].next_id.begin(),lane_list[i].next_id.end());
        // lane_list[i].next_id.erase(it2,lane_list[i].next_id.end());
        // std::sort(lane_list[i].next_id.begin(),lane_list[i].next_id.end());

        // int after_size1 = lane_list[i].next_id.size();
        // std::cout << "(next)after  size: " << after_size1 << std::endl;
        // for (int j=0;j<after_size1;j++) std::cout << lane_list[i].next_id[j] << " ";
        // std::cout << std::endl;

        write_func("lane",i);
        std::cout << "-----------------" << std::endl;

    }

    std::cout << "********* The Cross list is as followed *************" << std::endl;
    for(int i=0;i<101;i++){
        if(cross_list[i].path.poses.empty()) continue;
        // pre
        std::cout << "cross id: " << i << std::endl;
        int before_size = cross_list[i].pre_id.size();
        std::cout << "(pre)before size: " << before_size << std::endl;
        for (int j=0;j<before_size;j++) std::cout << cross_list[i].pre_id[j] << " ";
        std::cout << std::endl;

        // std::vector<int>::iterator it1 = std::unique(cross_list[i].pre_id.begin(),cross_list[i].pre_id.end());
        // cross_list[i].pre_id.erase(it1,cross_list[i].pre_id.end());
        // std::sort(cross_list[i].pre_id.begin(),cross_list[i].pre_id.end());

        // int after_size = cross_list[i].pre_id.size();
        // std::cout << "(pre)after  size: " << after_size << std::endl;
        // for (int j=0;j<after_size;j++) std::cout << cross_list[i].pre_id[j] << " ";
        // std::cout << std::endl << std::endl;
        
        // next
        int before_size1 = cross_list[i].next_id.size();
        std::cout << "(next)before size: " << before_size1 << std::endl;
        for (int j=0;j<before_size1;j++) std::cout << cross_list[i].next_id[j] << " ";
        std::cout << std::endl;

        // std::vector<int>::iterator it2 = std::unique(cross_list[i].next_id.begin(),cross_list[i].next_id.end());
        // cross_list[i].next_id.erase(it2,cross_list[i].next_id.end());
        // std::sort(cross_list[i].next_id.begin(),cross_list[i].next_id.end());

        // int after_size1 = cross_list[i].next_id.size();
        // std::cout << "(next)after  size: " << after_size1 << std::endl;
        // for (int j=0;j<after_size1;j++) std::cout << cross_list[i].next_id[j] << " ";
        // std::cout << std::endl;

        write_func("cross",i);
        std::cout << "-----------------" << std::endl;
    }

}

void WHOLE_PATH::extract_path::write_func(const std::string path_type, const int id){
    std::stringstream file_name;
    file_name << path_prefix << path_type << "_" << id << ".csv";
    std::ofstream f;
    f.open(file_name.str().c_str(),std::ios::out);
    if(!f.is_open()){
        ROS_WARN_STREAM("Can not open file to save.");
        std::cout << "Check: " << file_name.str() << std::endl;
        return;
    }
    
    // 由于Lane和Cross格式可能不一样,因此最好分别单独存储
    if(std::strcmp(path_type.c_str(),"lane")==0){
        f << id << std::endl
          << lane_list[id].length << std::endl;
        if(lane_list[id].reverse){
            f << 1 << std::endl;
        }else{
            f << 0 << std::endl;
        }

        for(int i=0;i<lane_list[id].pre_id.size();i++){
            f << lane_list[id].pre_id[i];
            if (i < lane_list[id].pre_id.size()-1) f << ",";
        }
        f << std::endl;

        for(int i=0;i<lane_list[id].next_id.size();i++){
            f << lane_list[id].next_id[i];
            if (i < lane_list[id].next_id.size()-1) f << ",";
        }
        f << std::endl;

        for(int i=0;i<lane_list[id].path.poses.size();i++){
            f << lane_list[id].path.poses[i].pose.position.x << ","
              << lane_list[id].path.poses[i].pose.position.y << ","
              << lane_list[id].path.poses[i].pose.position.z << ","
              << lane_list[id].path.poses[i].pose.orientation.x << ","
              << lane_list[id].path.poses[i].pose.orientation.y << ","
              << lane_list[id].path.poses[i].pose.orientation.z << ","
              << lane_list[id].path.poses[i].pose.orientation.w << std::endl;
        }
    }
    else{
        f << id << std::endl
          << cross_list[id].length << std::endl;
        if(cross_list[id].reverse){
            f << 1 << std::endl;
        }else{
            f << 0 << std::endl;
        }

        for(int i=0;i<cross_list[id].pre_id.size();i++){
            f << cross_list[id].pre_id[i];
            if (i < cross_list[id].pre_id.size()-1) f << ",";
        }
        f << std::endl;

        for(int i=0;i<cross_list[id].next_id.size();i++){
            f << cross_list[id].next_id[i];
            if (i < cross_list[id].next_id.size()-1) f << ",";
        }
        f << std::endl;

        for(int i=0;i<cross_list[id].path.poses.size();i++){
            f << cross_list[id].path.poses[i].pose.position.x << ","
              << cross_list[id].path.poses[i].pose.position.y << ","
              << cross_list[id].path.poses[i].pose.position.z << ","
              << cross_list[id].path.poses[i].pose.orientation.x << ","
              << cross_list[id].path.poses[i].pose.orientation.y << ","
              << cross_list[id].path.poses[i].pose.orientation.z << ","
              << cross_list[id].path.poses[i].pose.orientation.w << std::endl;
        } 
    }
    f.close();
    std::cout << "Write back successfully: " << file_name.str() << std::endl;
}


int main(int argc,char** argv){
    ros::init(argc,argv,"test_nav_msgs");
    WHOLE_PATH::extract_path app;
    app.init();
    ros::spin();
    return 0;
}