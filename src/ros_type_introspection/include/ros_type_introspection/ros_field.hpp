/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016-2017 Davide Faconti
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* *******************************************************************/


#ifndef ROS_INTROSPECTION_ROSFIELD_H
#define ROS_INTROSPECTION_ROSFIELD_H

#include <vector>
#include <map>
#include <iostream>
#include "ros_type_introspection/ros_type.hpp"

namespace RosIntrospection{

class ROSMessage;

class ROSField {
public:
  ROSField(const std::string& definition);
  const std::string& Name() const { return fieldname_; }   //变量名
  const ROSType&  Type() const { return type_; }
  bool IsConstant() const {
    return value_.size() != 0;
  }
  const std::string& Value()  const   { return value_; }
  bool IsArray() const { return arraySize_ != 1; }
  int  ArraySize() const { return arraySize_; }
  friend class ROSMessage;
protected:
  std::string fieldname_;
  ROSType     type_;
  std::string value_;
  int arraySize_;
};

}

#endif
