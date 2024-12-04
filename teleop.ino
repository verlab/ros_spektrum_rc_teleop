#include "SpektrumSatellite.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

float Z_CONSTANT = 1.4;
float X_FAST_CONSTANT = 2.0;
float X_SLOW_CONSTANT = 0.4;

int CHANNEL_ID_JOY_UPD = 4;
int CHANNEL_ID_JOY_RL = 3;
int CHANNEL_ID_GEAR = 9;
int CHANNEL_ID_MIX = 12;

ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;
ros::Publisher velpub("/jackal_velocity_controller/cmd_vel" ,&twist_msg);

SpektrumSatellite<uint16_t> satellite(Serial2); // Assing satellite to Serial (use Serial1 or Serial2 if available!)

float applyDeadZone(float value, float threshold) {
    if (abs(value) < threshold) {
        return 0.0;
    }
    return value;
}


void setup() {
  Serial2.begin(SPEKTRUM_SATELLITE_BPS);
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");

  // Activate the loggin to the console only if SpektrumSatellite is not using Serial
  satellite.setLog(Serial);
  // we can define the requested binding mode
  satellite.setBindingMode(External_DSMx_22ms);

  // wait forever for data
  satellite.waitForData();
  // init publisher
  nh.initNode();
  nh.advertise(velpub);



  
}

void loop() {
  
  if (satellite.getFrame()) { 

    Channel stop = static_cast<Channel>(CHANNEL_ID_MIX);
    long stop_value = satellite.getChannelValue(stop);
    bool stop_now = stop_value>=700;

    if(!stop_now){

      Channel gear = static_cast<Channel>(CHANNEL_ID_GEAR);
      Channel up_down = static_cast<Channel>(CHANNEL_ID_JOY_UPD);
      Channel left_right =  static_cast<Channel>(CHANNEL_ID_JOY_RL);
      
      long gear_value = satellite.getChannelValue(gear);
      long x = satellite.getChannelValue(up_down);
      long z = satellite.getChannelValue(left_right);
      float x_normalized = ((x - 443) / (1629.0 - 443.0)) * 2.0 - 1.0;
      float z_normalized = ((z - 422) / (1626.0 - 422.0)) * 2.0 - 1.0;

      bool vel_mode = gear_value>=700;

      x_normalized = applyDeadZone(x_normalized, 0.06);
      z_normalized = applyDeadZone(z_normalized, 0.06);

      if(vel_mode){
        twist_msg.linear.x = X_FAST_CONSTANT*x_normalized;
        twist_msg.angular.z = Z_CONSTANT*z_normalized;

      }
      else {
        twist_msg.linear.x = X_SLOW_CONSTANT*x_normalized;
        twist_msg.angular.z = Z_CONSTANT*z_normalized;
      }
      velpub.publish(&twist_msg);
      nh.spinOnce();
    }
  }
}