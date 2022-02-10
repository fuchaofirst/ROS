/**
 * @file rostopic_echo.cpp
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2018. All rights reserved.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <cstdio>
#include <iomanip>
#include <thread>
#include <regex>
#include <cmath>
#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "ros_type_introspection/ros_introspection.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include "shape_shifter.h"
#include "ros_type_introspection/ros_introspection.hpp"
const uint32_t echoTopicMsgCnt_ = 0;
std::string ParserHeader(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topicName,
                         RosIntrospection::Parser& parser)
{
    std::pair<std::string, std::string> topicInfo {};//RetInfoTopic(topicName);
    std::string dataType = "std_msgs/String";
    std::string definition = "string data\n";
    //std::cout<<dataType_  <<"--------->"<<definition_<<std::endl;
    if ((definition == "") || (dataType == "")) {
        std::cout << "data type is empty!" << std::endl;
        return "";
    }
    parser.RegisterMessageDefinition(topicName, RosIntrospection::ROSType(dataType), definition);
    std::vector<uint8_t> buffer;
    RosIntrospection::FlatMessage flat_container;
    buffer.resize(msg->size());
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);
    parser.DeserializeIntoFlatContainer(topicName, (buffer), &flat_container,200 );//msg->size()
    return "  ";
}

void CallBackFun(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    RosIntrospection::Parser parser;
    const std::string topicName;
    std::string msgInfo = ParserHeader(msg, topicName, parser);
    if (msgInfo.empty()) {
        std::cout<<"get massge info faild for echo command!"<<std::endl;
        return;
    }
    std::cout<<msgInfo<<std::endl;
    std::cout << "\n======================================" << std::endl;

}
int main(int argc, char* argv[])
{
    std::string topicName = "/chatter";
    std::cout<<"need parase topicName is "<<topicName<<std::endl;
    ros::init(argc, argv, "DfxTopicTool", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(topicName, 1000,CallBackFun);
    ros::spin();

}
