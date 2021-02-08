#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#define RAD2DEG 57.295779513
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// for stoppşng and restarting the car (failsafe)
int getkey();
bool work = true;


// ros depth callback
void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

	float columnsAverage[90];

	
	//float sum = 0;
	float sum = 1;
	int angle = 0;
	int angle_to_pixel = 0;

	for (int i = 0; i < 1280; i++){
		for (int j = -50; j < 50; j++){
		
			float current_point = depths[i + msg->width * (u + j)];

			if(current_point == -INFINITY || current_point == INFINITY || isnan(current_point) || current_point > 20 || current_point < 0){
				// skip			
				continue;
			} // if
			
				sum+= current_point;
			
		} // for j

		float avreage = sum/(100);

		//printf("sum %f \n", sum);
		columnsAverage[i] = sum;		
		sum = 1;
	}// for i



	int max_column = 0;
	char x;

	// find max column
	for (int i = 1; i < 90; i++)
	{
		if (columnsAverage[i] > columnsAverage[max_column])
			max_column = i;
	} 


	// char 0 = 48
	enum Kara {dur = 48, ileri ,sag, sol};

    if(!work){
        x = dur;
        ROS_INFO("DUR!!!");
    }
	else if (max_column < 340){
        x = sol;
        ROS_INFO("SOL");
    }
    else if (max_column < 680){
        x = ileri;
        ROS_INFO("ILERI");
    }
    else{
        x = sag;
        ROS_INFO("SAG");
    }


	struct termios tty;


    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Kara gonderme icin
    char car[] = { 0 };
    car[0] = x;

	// Opeining the serial port
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattrASRFASFASF: %s\n", errno, strerror(errno));
        return;
    }

    // Write to serial port
    write(serial_port, car, 1);
    //write(serial_port, yaw, 1);

    //press b to stop the arac and v to restart
	// used as a fail safe manual control
    if(getkey()==98){
	work = false;
   }
    if(getkey()==118){
	work = true;
   }
close(serial_port);


}

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_video_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subDepth = n.subscribe("/zed/zed_node/depth/depth_registered", 10, depthCallback);

    ros::spin();

    return 0;
}


        
        
