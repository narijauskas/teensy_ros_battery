#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>


const int voltage_pin = 0;
int analog_value;
float voltage;

ros::NodeHandle nh;

std_msgs::Float32 msg;
ros::Publisher battery_voltage("battery_voltage", &msg);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //while(!Serial);//wait for serial to initialize

  nh.initNode();
  nh.advertise(battery_voltage);
}

void loop() {
  // put your main code here, to run repeatedly:
  analog_value = analogRead(voltage_pin);
  voltage = (analog_value*33.0)/1024;
  //Serial.println(voltage);

  msg.data = voltage;
  battery_voltage.publish( &msg );
  nh.spinOnce();
  delay(1000);
}