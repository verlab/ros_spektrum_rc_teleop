#include "SpektrumSatellite.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>


int CHANNEL_ID_JOY_UPD = 4;
int CHANNEL_ID_JOY_RL = 3;
int CHANNEL_ID_GEAR = 8;
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;
ros::Publisher velpub("/jackal_velocity_controller/cmd_vel" ,&twist_msg);

SpektrumSatellite<uint16_t> satellite(Serial2); // Assing satellite to Serial (use Serial1 or Serial2 if available!)
int counter = 0;

void setup() {
  Serial2.begin(SPEKTRUM_SATELLITE_BPS);
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");

  // Activate the loggin to the console only if SpektrumSatellite is not using Serial
  satellite.setLog(Serial);
  // we can define the requested binding mode
  satellite.setBindingMode(External_DSMx_22ms);

  // //scale the values from 0 to 180 degrees for PWM
  // satellite.setChannelValueRange(0, 180);
  // wait forever for data
  satellite.waitForData();
  nh.initNode();
  nh.advertise(velpub);



  
}

void loop() {
  
  if (satellite.getFrame()) {   
    Channel gear = static_cast<Channel>(CHANNEL_ID_GEAR);
    Channel up_down = static_cast<Channel>(CHANNEL_ID_JOY_UPD);
    Channel left_right =  static_cast<Channel>(CHANNEL_ID_JOY_RL);
    
    long gear_value = satellite.getChannelValue(gear);
    long x = satellite.getChannelValue(up_down);
    long z = satellite.getChannelValue(left_right);
    float x_normalized = (x-443)/(1629.0-443.0)-0.5;
    float z_normalized = (z-422)/(1626-422.0)-0.5;

    bool vel_mode = gear_value>=700;

    if (abs(x_normalized) < 0.01) {
        x_normalized = 0.0;
    }
    else if (abs(z_normalized) < 0.01) {
        z_normalized = 0.0;
    }
    if(vel_mode){
      twist_msg.linear.x = 2*x_normalized;
      twist_msg.angular.z = 2*z_normalized;

    }
    else if(!vel_mode){
      twist_msg.linear.x = x_normalized;
      twist_msg.angular.z = z_normalized;
    }
    velpub.publish(&twist_msg);
    nh.spinOnce();

  }
}