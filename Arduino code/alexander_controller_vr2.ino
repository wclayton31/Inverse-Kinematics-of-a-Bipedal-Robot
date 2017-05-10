#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <alexander_arduino_communication/sequence.h>
#include <alexander_arduino_communication/sequence_vector.h>
#include <alexander_arduino_communication/walking_engine_creation.h>

String position_sequence[100];
int delay_time[100];
int number_of_sequences = 0;

ros::NodeHandle nh;

using alexander_arduino_communication::walking_engine_creation;

ros::ServiceClient<walking_engine_creation::Request, walking_engine_creation::Response> client("walking_engine_creation_srv");

std_msgs::String str_msg;
std_msgs::Empty empty_msg;

void request_sequence(const std_msgs::Empty& get_sequence_msg)
{
  nh.loginfo("Reading sequence in...");
  walking_engine_creation::Request req;
  walking_engine_creation::Response res;
  
  req.distance = 0;
  req.step_number = 0;
  req.level_terain = 0;
  
  client.call(req, res);
  nh.loginfo("Sequence recived");
  
  number_of_sequences = res.sequence.vector_len;
  for(int i=0; i<number_of_sequences; i++)
  {
    position_sequence[i] = res.sequence.vector_sequence[i].sequence;
    delay_time[i] = res.sequence.vector_sequence[i].delay_number;
  }
}

void run_current_sequence(const std_msgs::Empty& run_sequence_msg)
{
  nh.loginfo("Sending Seequence");
  
  for(int i=0; i<=number_of_sequences; i++)
  {
    Serial1.println(position_sequence[i]);
    delay(delay_time[i]);
  }
}

ros::Subscriber<std_msgs::Empty> get_sequence("get_sequence", &request_sequence);
ros::Subscriber<std_msgs::Empty> run_sequence("run_sequence", &run_current_sequence);

void setup() 
{
  Serial1.begin(115200);
  nh.initNode();
  
  nh.subscribe(get_sequence);
  nh.subscribe(run_sequence);
  
  nh.serviceClient(client);
  
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("*******************************");
  nh.loginfo("Startup complete");
  nh.loginfo("*******************************");
}

void loop() 
{
  nh.spinOnce();
  delay(100);
}
