#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <SimpleTimer.h>

//these variables hold the raw values sampled from the ADC and calculate a running average
const int sample_pin = 9;   //the analog pin to which the battery monitor is attached
const int num_samples = 20; //how many samples to consider for the running average
int samples[num_samples];   //an array of samples
int sample_index = 0;       //the index of the current sample
int sample_total = 0;       //the running total

float average_voltage = 0.0; //the calculated value of voltage

//these variables are for blinking the led
const int led_pin = 14; //note this is a digital pin
bool led_on;            //keep track of whether the LED is on or off
int8_t e_stop_status = 0; //0 = normal, no e-stop 1 = e-stop, blink led, 2 = safe, led on

//timer stuff
SimpleTimer timer; //a timer object to control how often we publish to the ros topic
const int timer_interval = 1000; //how often the timer callback runs (ms)


void message_cb( const std_msgs::Int8& s_msg)
{
    if(s_msg.data == 0)//if e_stop is disabled
    {
        digitalWrite(led_pin, LOW);   //turn off the LED
        led_on = false;
        e_stop_status = 0;
    }

    if(s_msg.data == 1)//if e_stop has been triggered, and the quad is unsafe
    {
        e_stop_status = 1;   //the LED will blink via the timer callback
    }

    if(s_msg.data == 2)//if e_stop is has been triggered and the quad has landed
    {
        digitalWrite(led_pin, HIGH);   //turn on the LED
        led_on = true;
        e_stop_status = 2;
    }
    
}

//these objects are used to set up a publisher/subscriber ros node
ros::NodeHandle nh;
std_msgs::Float32 p_msg;
ros::Publisher pub("battery_voltage", &p_msg);
ros::Subscriber<std_msgs::Int8> sub("e_stop", &message_cb);


void timer_cb()
{
    //publish the average voltage as a ros topic
    p_msg.data = average_voltage;
    pub.publish( &p_msg );
    nh.spinOnce();


    //change the state of the LED if e-stop in unsafe mode
    if ((e_stop_status == 1) & led_on)
    {
        //if the led is on, turn it off
        digitalWrite(led_pin, LOW);
        led_on = false;
    }
    else if (e_stop_status == 1)
    {
        //otherwise turn it on
        digitalWrite(led_pin, HIGH);
        led_on = true;
    }
}


void setup()
{
    //this code runs once:

    analogReadResolution(12); //set the analog read resolution to 12-bit (4026)

    //initialize the digital LED pin as an output, turn the LED off
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    led_on = false;
    
    // initialize all the values in the sample array to 0:
    for (int ix = 0; ix < num_samples; ix++)
    {
        samples[ix] = 0;
    }
    
    //start the ROS node
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);

    //set the timer object to run the timer_callback method
    timer.setInterval(timer_interval, timer_cb);
}

void loop()
{
    //this is the main loop that runs continuously 
    //it samples the voltage often, and keeps a running average to smooth the data

    timer.run();//necessary to include for timer functionality

    
    sample_total = sample_total - samples[sample_index]; // subtract the last sample from the total
    samples[sample_index] = analogRead(sample_pin); // read a new sample from the sensor
    sample_total = sample_total + samples[sample_index]; // add the new sample to the total: (in place of the one that got removed)
    sample_index = sample_index + 1; // increment the sample index

    if (sample_index >= num_samples)// if we're at the end of the array...
    { 
        sample_index = 0; // ...wrap around to the beginning
    }

    average_voltage = (sample_total*33.0)/(4026*num_samples); //calculate the average voltage from the samples
    
    delay(100);
}