/*!
 * 9DOF ArduIMU node - Ricardo Ragel
 * ---------------------------------
 * 
 * This node read the IMU attitude (Yaw+Pitch+Roll) output in format: 
 * 	[#YPR=][-][000][.][00][,] ...
 * 
 * and publish it through the ROS topic "arduimu/rpy"
 * 
 * Serial port configuration must be 115200, 8N1, NOR
 */

#include <signal.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <serial_utils.h>

/// Un-comment to get each debug information
//~ #define DEBUG_INCOMING_SERIAL_PORT_DATA
//~ #define DEBUG_PARSED_DATA

int fdes;
bool exit_;

void SigintHandler(int sig)
{
  ROS_ERROR("arduimu node: Receiving Signal, exiting...");
  exit_=true;
  close(fdes);
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "arduimu");
  ros::NodeHandle n;
  ros::NodeHandle n_params("~");
  
  signal(SIGINT, SigintHandler);
  
  ros::Publisher imu_pub = n.advertise<geometry_msgs::Vector3>("/arduimu/rpy",1);

  std::string s;
  n_params.param<std::string>("portname", s, "/dev/ttyUSB0");
  const char * portname = s.c_str();
  fdes = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdes < 0)
  {
        ROS_ERROR("Error %d opening %s: %s", errno, portname, strerror (errno));
        return 1;
  } 
  set_interface_attribs (fdes, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
  set_blocking (fdes, 1);                    // set blocking

  char buf[1];
  int nc, i;

  tcflush( fdes, TCIFLUSH );
  ros::Time last_time=ros::Time::now();
  exit_=false;
  
  // Flush the buffer to get the most recent data   
  tcflush( fdes, TCIFLUSH );
      
  // Reading while: format of the serial data: [#YPR=][-][000][.][00][,] ...
  ROS_INFO("arduimu: Ready to read from %s!", portname);
  float yaw = 0.0, pitch= 0.0, roll= 0.0;
  while(ros::ok() && !exit_)
  { 
	  nc = read (fdes, buf, 1); 
	  #ifdef DEBUG_INCOMING_SERIAL_PORT_DATA
		printf("nc: %d, BUF: %c\n", nc, buf[0]);
	  #else
	  // Wait for first character: a line break
	  if((nc==1) && buf[0]=='\n')
	  {
		#ifdef DEBUG_PARSED_DATA
			printf("Ampersand Read!\n");
		#endif
		
		// Init vars to zero
		yaw = 0.0;
		pitch = 0.0;
		roll = 0.0;
		
		/*
		 *	YAW
		 */

		// Read yaw integer part
		i = 1;
		bool yaw_negative = false;
		do
		{ 
		   nc = read (fdes, buf, 1); 
		   if ((nc==1) && (isValidNumber(buf[0])))
		   {
			 yaw=yaw*i+atof(buf);
			 i=10;
		   }
		   else if(buf[0] =='-')
		   {
			   yaw_negative = true;
		   }
		   else if(buf[0] =='.')
		   {
			 break;
		   }
		}
		while(ros::ok() && !exit_);
		//printf("Yaw: %f\n", yaw);
		
		// Read the two decimals of yaw number
		nc = read (fdes, buf, 1); 
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  yaw=0.0;
		  ROS_ERROR("Error reading firt distance decimal value");
		}	
		yaw=yaw+atof(buf)/10.0;
		nc = read (fdes, buf, 1);
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  yaw=0.0;
		  ROS_ERROR("Error reading second distance decimal value");
		}
		yaw=yaw+atof(buf)/100.0;
		
		// Set negative if a '-' was read
		if(yaw_negative)
			yaw=yaw*(-1.0);
		
		#ifdef DEBUG_PARSED_DATA
			printf("Yaw: %f\n", yaw);
		#endif

		/*
		 *	PITCH
		 */

		// Read pitch integer part
		i = 1;
		bool pitch_negative = false;
		do
		{ 
		   nc = read (fdes, buf, 1); 
		   if ((nc==1) && (isValidNumber(buf[0])))
		   {
			 pitch=pitch*i+atof(buf);
			 i=10;
		   }
		   else if(buf[0] =='-')
		   {
			   pitch_negative = true;
		   }
		   else if(buf[0] =='.')
		   {
			 break;
		   }
		}
		while(ros::ok() && !exit_);
		//printf("Pitch: %f\n", pitch);
		
		// Read the two decimals of pitch number
		nc = read (fdes, buf, 1); 
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  pitch=0.0;
		  ROS_ERROR("Error reading firt distance decimal value");
		}	
		pitch=pitch+atof(buf)/10.0;
		nc = read (fdes, buf, 1);
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  pitch=0.0;
		  ROS_ERROR("Error reading second distance decimal value");
		}
		pitch=pitch+atof(buf)/100.0;
		
		// Set negative if a '-' was read
		if(pitch_negative)
			pitch=pitch*(-1.0);
		
		#ifdef DEBUG_PARSED_DATA
			printf("Pitch: %f\n", pitch);
		#endif

		/*
		 *	Roll 
		 */

		// Read roll integer part
		i = 1;
		bool roll_negative = false;
		do
		{ 
		   nc = read (fdes, buf, 1); 
		   if ((nc==1) && (isValidNumber(buf[0])))
		   {
			 roll=roll*i+atof(buf);
			 i=10;
		   }
		   else if(buf[0] =='-')
		   {
			   roll_negative = true;
		   }
		   else if(buf[0] =='.')
		   {
			 break;
		   }
		}
		while(ros::ok() && !exit_);
		//printf("Roll: %f\n", roll);
		
		// Read the two decimals of roll number
		nc = read (fdes, buf, 1); 
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  roll=0.0;
		  ROS_ERROR("Error reading firt distance decimal value");
		}	
		roll=roll+atof(buf)/10.0;
		nc = read (fdes, buf, 1);
		if ((nc!=1) || (!isValidNumber(buf[0])))
		{
		  roll=0.0;
		  ROS_ERROR("Error reading second distance decimal value");
		}
		roll=roll+atof(buf)/100.0;
		
		// Set negative if a '-' was read
		if(roll_negative)
			roll=roll*(-1.0);
		
		#ifdef DEBUG_PARSED_DATA
			printf("Roll: %f\n", roll);
		#endif

		// Publish to ROS topic
		geometry_msgs::Vector3 rpy;
		rpy.x = roll;  
		rpy.y = pitch;  
		rpy.z = yaw;  
		imu_pub.publish(rpy);

	  } // if 'linebreak'
	  #endif // end else of #ifdef DEBUG_INCOMING_SERIAL_PORT_DATA
  } // reading while 
}
