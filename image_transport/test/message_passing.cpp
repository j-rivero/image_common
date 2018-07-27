/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2018 Open Robotics
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <gtest/gtest.h>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"

int total_images_received = 0;

class MessagePassingTesting : public ::testing::Test {
  public:
    image_transport::ImageTransport it()
    {
      return image_transport::ImageTransport(nh_);
    }

    sensor_msgs::ImagePtr generate_random_image()
    {
      cv::Mat img(100, 100, CV_8UC3);
      cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
      return cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    }

  protected:
    void SetUp()  {
      total_images_received = 0;
    }

    ros::NodeHandle nh_;
};


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    total_images_received++;
}

TEST_F(MessagePassingTesting, one_message_passing)
{
    image_transport::Publisher pub = it().advertise("camera/image", 1);
    image_transport::Subscriber sub = it().subscribe("camera/image", 1, imageCallback);

    // generate random image and publish it
    pub.publish(generate_random_image());
    ros::spinOnce();

    ASSERT_EQ(1, total_images_received);
}

TEST_F(MessagePassingTesting, stress_message_passing)
{
  int images_to_stress = 1000;

  image_transport::Publisher pub = it().advertise("camera/image", 1);
  image_transport::Subscriber sub = it().subscribe("camera/image", 1, imageCallback);

  // generate random image and publish it
  int image_pubs = 0;
  while (image_pubs < images_to_stress)
  {
    pub.publish(generate_random_image());
    ros::spinOnce();
    image_pubs++;
  }
    
  ASSERT_EQ(images_to_stress, total_images_received);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //Initialize ROS
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  int ret = RUN_ALL_TESTS();

  return ret;
};

