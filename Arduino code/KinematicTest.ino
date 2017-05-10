/*
*  William Clayton
* Servo Tester:
*/

#include<ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void event(const std_msgs::String& servo);
ros::Subscriber<std_msgs::String> sub("kinematics", event);
std_msgs::String str_msg;
ros::Publisher alexander("alexander", &str_msg);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(alexander);

  Serial.begin(115200);
  Serial1.begin(115200);
  
  delay(1000);
  Serial1.println("#0P1560 #1P1510 #2P1540 #3P1480 #4P1540 #5P1790 #6P1510 #7P1450 #8P1460 #9P1540 T2000");
  delay(2000);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

void event(const std_msgs::String& servo)
{
  str_msg.data = servo.data;
  alexander.publish(&str_msg);

  String data = servo.data;
  Serial1.println(servo.data);
}

