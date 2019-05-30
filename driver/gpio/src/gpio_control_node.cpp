#include "ros/ros.h"
#include "ros_gpio_control/gpio.h"
#include "std_msgs/Bool.h"
#include "jetsonGPIO.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_single");
    int gpio_number;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("gpio_number", gpio_number, 254);
    //nh_private.param("gpio_number", gpio_number, 255);
    //jetsonTX2GPIONumber trig = gpio397; // Output
    //gpioExport(trig);
    //gpioSetDirection(trig, outputPin);
    
    jetsonTX2GPIONumber echo = gpio254; // Input

    gpioExport(echo);

    gpioSetDirection(echo, inputPin);
    
    
    GPIO ultrasonic_single(gpio_number);
    
    ultrasonic_single.edge(GPIO::EDGE_BOTH);
    //ultrasonic_single.edge(GPIO::EDGE_RISE);

    ros::Publisher ultrasonic_pub = nh.advertise<std_msgs::Bool>("ultrasonic_single", 10);

    
    //ros::Rate loop_rate(60);

    unsigned int value = 1;
    
    while (ros::ok()) {
    
        ultrasonic_single.poll(0);
        
        //gpioGetValue(echo, &value);
        std_msgs::Bool msg;
       
        msg.data = ultrasonic_single.read();
        
        //msg.data = value;
        
        ultrasonic_pub.publish(msg);
        
        printf("read: %d \n", int(msg.data));
        

        if(ultrasonic_single.poll_targets.revents & POLLPRI)
        {
            printf("**********************************\n");
        }
        
        //ros::spinOnce();
        //loop_rate.sleep();
    }
    return 0;
}
