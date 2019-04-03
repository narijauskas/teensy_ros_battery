#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <SimpleTimer.h>

//these variables hold the raw values sampled from the ADC and calculate a running average
const int sample_pin = 9;   //the analog pin to which the battery monitor is attached
const int num_samples = 20; //how many samples to consider for the running average
int samples[num_samples];   //an array of samples
int sample_index = 0;       //the index of the current sample
int sample_total = 0;       //the running total

float average_voltage = 0.0; //the calculated value of voltage

//these variables are for blinking the led (so we know the teensy is on and working)
const int led_pin = 13; //note this is a digital pin
bool led_on;            //keep track of whether the LED is on or off

//these objects are used to set up a ros node that publishes to a topic called battery_voltage
ros::NodeHandle nh;
std_msgs::Float32 msg;
ros::Publisher battery_voltage("battery_voltage", &msg);

SimpleTimer timer; //a timer object to control how often we publish to the ros topic

void publish_to_ros()
{
    //publish the average voltage as a ros topic
    //runs as an interrupt called by the timer object
    msg.data = average_voltage;
    battery_voltage.publish( &msg );
    nh.spinOnce();


    //change the state of the LED
    if (led_on)
    {
        //if the led is on, turn it off
        digitalWrite(led_pin, LOW);
        led_on = false;
    }
    else
    {
        //otherwise turn it on
        digitalWrite(led_pin, HIGH);
        led_on = true;
    }
}

void setup()
{
    //this code runs once:

    //set the analog read resolution to 12-bit (4026)
    analogReadResolution(12);

    //initialize the digital LED pin as an output, turn the LED on
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);
    led_on = true;
    
    // initialize all the values in the sample array to 0:
    for (int ix = 0; ix < num_samples; ix++)
    {
        samples[ix] = 0;
    }
    
    //start the ROS node
    nh.initNode();
    nh.advertise(battery_voltage);

    //set the timer object to run the publish_to_ros method as an interrupt every second
    timer.setInterval(1000, publish_to_ros);
}

void loop()
{
    //this is the main loop that runs continuously 
    //it samples the voltage often, and keeps a running average to smooth the data

    timer.run();//necessary to include for timer functionality

    // subtract the last sample from the total:
    sample_total = sample_total - samples[sample_index];
    // read a new sample from the sensor:
    samples[sample_index] = analogRead(sample_pin);
    // add the new sample to the total: (in place of the one that got removed)
    sample_total = sample_total + samples[sample_index];
    
    // increment the sample index:
    sample_index = sample_index + 1;

    // if we're at the end of the array...
    if (sample_index >= num_samples)
    {
        // ...wrap around to the beginning:
        sample_index = 0;
    }

    //calculate the average voltage from the samples
    average_voltage = (sample_total*33.0)/(4026*num_samples);
    
    delay(100);
}