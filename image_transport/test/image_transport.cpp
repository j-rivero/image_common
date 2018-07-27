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
#include "image_transport/image_transport.h"

class ImageTransportTesting : public ::testing::Test {
  public:
    image_transport::ImageTransport it()
    {
      return image_transport::ImageTransport(nh_);
    }

  protected:
    ros::NodeHandle nh_;
};

void CHECK_TOPIC_EXISTS(const std::string & topic_name)
{
   ros::master::V_TopicInfo master_topics;
   EXPECT_TRUE(ros::master::getTopics(master_topics));
   bool ret = false;

   for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); ++it) {
     const ros::master::TopicInfo& info = *it;
     if (info.name == topic_name)
         ret = true;
   }

   EXPECT_TRUE(ret);
}

void CHECK_TOTAL_NUMBER_OF_TOPICS(const int number_to_check)
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it(nh_);

   ros::master::V_TopicInfo master_topics;
   EXPECT_TRUE(ros::master::getTopics(master_topics));

   int topic_count = 0;
   std::vector<std::string> transports = it.getLoadableTransports();

   for(std::vector<std::string>::const_iterator i = transports.begin();
     i != transports.end(); ++i) {
     if (*i == "image_transport/raw")
       // raw is just the raw topic
       topic_count++;
     else
       // theora, imageDepth and imageDepthCompressed has three pub 
       // topics each one
       topic_count = topic_count + 3;
   }

   // rostest topic out. topics expected + 3 *
   EXPECT_EQ(master_topics.size() - 1, number_to_check * topic_count);
}

TEST(ImageTransport, constructor_destructor)
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport object(nh_);
}

TEST_F(ImageTransportTesting, advertise_simple)
{
    image_transport::Publisher pub = it().advertise("foo", 2);
    CHECK_TOPIC_EXISTS("/foo");
    CHECK_TOTAL_NUMBER_OF_TOPICS(1);
}

TEST_F(ImageTransportTesting, advertise_callback)
{
    image_transport::Publisher pub = it().advertise("out_image_base_topic", 1);
    CHECK_TOPIC_EXISTS("/out_image_base_topic");
    CHECK_TOTAL_NUMBER_OF_TOPICS(1);
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

