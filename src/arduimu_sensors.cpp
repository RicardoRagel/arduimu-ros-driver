/*!
 * 9DOF ArduIMU node - Ricardo Ragel
 * ---------------------------------
 * 
 * This node read the IMU sensors (accel, gyros, magnet) output in format:
 * 	#M-C=17.83,-25.33,-42.83
 * 	#G-C=-32.00,13.00,-3.00
 *	#A-C=-73.73,-174.08,160.77
 * 
 * and publish them through the ROS topic "/arduimu/accel", "/arduimu/gyros", /arduimu/magne"
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
    
  ros::Publisher accel_pub, gyros_pub, magne_pub;
  accel_pub = n.advertise<geometry_msgs::Vector3>("/arduimu/accel",1);
  gyros_pub = n.advertise<geometry_msgs::Vector3>("/arduimu/gyros",1);
  magne_pub = n.advertise<geometry_msgs::Vector3>("/arduimu/magne",1);

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

  // Serial data
  char buf[1];
  int nc, i;

  // Sensors data
  float ax, ay, az;
  float gx, gy, gz;
  float mx,	my, mz;
  
  // Sensors data flags
  bool accel_read = false;
  bool magne_read = false;
  bool gyros_read = false;
  
  // Aux vars
  double now = ros::Time::now().toSec();
  double lastTime = ros::Time::now().toSec();

  tcflush( fdes, TCIFLUSH );
  ros::Time last_time=ros::Time::now();
  exit_=false;
  
  // Flush the buffer to get the most recent data   
  tcflush( fdes, TCIFLUSH );
      
  // Reading while: format of the serial data
  ROS_INFO("arduimu: Ready to read from %s!", portname);
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
			printf("Line break Read!\n");
		#endif
		
		// Read Hashtag
		nc = read (fdes, buf, 1);
		if((nc==1) && buf[0]=='#')
		{
			#ifdef DEBUG_PARSED_DATA
				printf("Hashtag Read!\n");
			#endif
			
			// Read each sensor header and, then, its data
			nc = read (fdes, buf, 1);
			if((nc==1))
			{
				bool ax_negative = false, ay_negative = false, az_negative = false;
				bool mx_negative = false, my_negative = false, mz_negative = false;
				bool gx_negative = false, gy_negative = false, gz_negative = false;
				
				switch(buf[0])
				{
					/*
					 * Accel
					 */
					case 'A':

						#ifdef DEBUG_PARSED_DATA
							printf("Accelerometer Read!\n");
						#endif
						
						// Read character until '='
						do
						{ 
							nc = read (fdes, buf, 1); 
						}
						while(buf[0] !='=');
						
						/*
						 * X
						 */
						ax = 0.0;
						
						// Read X integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								ax=ax*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								ax_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of ax number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							ax=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						ax=ax+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							ax=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						ax=ax+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(ax_negative)
							ax=ax*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("ax: %f\n", ax);
						#endif
						
						/* 
						 * Y
					     */
						ay = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								ay=ay*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								ay_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of ax number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							ay=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						ay=ay+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							ay=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						ay=ay+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(ay_negative)
							ay=ay*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("ay: %f\n", ay);
						#endif

						/*
						 * Z
						 */
						az = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								az=az*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								az_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of ax number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							az=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						az=az+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							az=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						az=az+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(az_negative)
							az=az*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("az: %f\n", az);
						#endif
						
						accel_read = true;
						break;

					/*
					 * Gyros
					 */
					case 'G':

						#ifdef DEBUG_PARSED_DATA
							printf("Gyroscope Read!\n");
						#endif
														
						// Read character until '='
						do
						{ 
							nc = read (fdes, buf, 1); 
						}
						while(buf[0] !='=');
						
						/*
							* X
							*/
						// Clean
						gx = 0.0;
						// Read X integer part
						i = 1;
						
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								gx=gx*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								gx_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of gx number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gx=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						gx=gx+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gx=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						gx=gx+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(gx_negative)
							gx=gx*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("gx: %f\n", gx);
						#endif
						
						/* 
							* Y
							*/
						// Clean
						gy = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								gy=gy*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								gy_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of gy number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gy=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						gy=gy+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gy=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						gy=gy+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(gy_negative)
							gy=gy*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("gy: %f\n", gy);
						#endif

						/*
							* Z
							*/
						// Clean
						gz = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								gz=gz*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								gz_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of gz number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gz=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						gz=gz+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							gz=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						gz=gz+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(gz_negative)
							gz=gz*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("gz: %f\n", gz);
						#endif
						
						gyros_read = true;
						break;
								
					/*
					 * Magnet
					 */
					case 'M':

						#ifdef DEBUG_PARSED_DATA
							printf("Magnetometer Read!\n");
						#endif
						
						// Read character until '='
						do
						{ 
							nc = read (fdes, buf, 1); 
						}
						while(buf[0] !='=');
						
						/*
							* X
							*/
						mx = 0.0;
						// Read X integer part
						i = 1;
						
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								mx=mx*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								mx_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of mx number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							mx=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						mx=mx+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							mx=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						mx=mx+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(mx_negative)
							mx=mx*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("mx: %f\n", mx);
						#endif
						
						/*
							* Y
							*/
						my = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								my=my*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								my_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of my number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							my=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						my=my+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							my=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						my=my+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(my_negative)
							my=my*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("my: %f\n", my);
						#endif

						/*
							* Z
							*/
						mz = 0.0;
						// Read Y integer part
						i = 1;
						do
						{ 
							nc = read (fdes, buf, 1); 
							if ((nc==1) && (isValidNumber(buf[0])))
							{
								mz=mz*i+atof(buf);
								i=10;
							}
							else if(buf[0] =='-')
							{
								mz_negative = true;
							}
							else if(buf[0] =='.')
							{
								break;
							}
						}
						while(ros::ok() && !exit_);
						
						// Read the two decimals of gz number
						nc = read (fdes, buf, 1); 
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							mz=0.0;
							ROS_ERROR("Error reading firt distance decimal value");
						}	
						mz=mz+atof(buf)/10.0;
						nc = read (fdes, buf, 1);
						if ((nc!=1) || (!isValidNumber(buf[0])))
						{
							mz=0.0;
							ROS_ERROR("Error reading second distance decimal value");
						}
						mz=mz+atof(buf)/100.0;
						
						// Set negative if a '-' was read
						if(mz_negative)
							mz=mz*(-1.0);
						
						#ifdef DEBUG_PARSED_DATA
							printf("mz: %f\n", mz);
						#endif
						
						magne_read = true;
						break;
					default:
						ROS_ERROR("ERROR READING SENSOR HEADER");
				}
			}
		}

		// Publish Accelerometer
		if(accel_read)
		{
			#ifdef DEBUG_PARSED_DATA
				printf("Publishing accel topic!\n");
			#endif
			
			geometry_msgs::Vector3 accel;
			accel.x = ax;
			accel.y = ay;
			accel.z = az;
			accel_pub.publish(accel);
			
			accel_read = false;
		}

		// Publish Gyros
		if(gyros_read)
		{
			#ifdef DEBUG_PARSED_DATA
				printf("Publishing gyros topic!\n");
			#endif
			
			geometry_msgs::Vector3 gyros;
			gyros.x = gx;
			gyros.y = gy;
			gyros.z = gz;
			gyros_pub.publish(gyros);

			gyros_read = false;
		}

		// Publish Magnet
		if(magne_read)
		{
			#ifdef DEBUG_PARSED_DATA
				printf("Publishing magnetometer topic!\n");
			#endif
			
			geometry_msgs::Vector3 magne;
			magne.x = mx;
			magne.y = my;
			magne.z = mz;
			magne_pub.publish(magne);

			magne_read = false;
		}
		
	  } // if 'linebreak'
	  #endif // end else of #ifdef DEBUG_INCOMING_SERIAL_PORT_DATA
  } // reading while 
}
