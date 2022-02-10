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

#ifndef ROS_INTROSPECTION_STRINGTREELEAF_H
#define ROS_INTROSPECTION_STRINGTREELEAF_H

#include <vector>
#include <map>
#include <iostream>
#include "ros_type_introspection/ros_message.hpp"

namespace RosIntrospection{


template <typename T, size_t N>
class InlinedVector{
public:
    InlinedVector(): size_(0) {}
    void push_back(T val) { array_[size_++] = val; }
    const T& back() const { return array_[size_-1]; }
    T& back()             { return array_[size_-1]; }
    size_t size() const { return size_; }
    const T& operator[](size_t index) const { return array_[index]; }
    T& operator[](size_t index)             { return array_[index]; }
private:
    std::array<T,N> array_;
    size_t size_;
};

struct StringTreeLeaf{

  StringTreeLeaf();

  const StringTreeNode* nodePtr;

  InlinedVector<uint16_t,8> indexarray;

  constexpr static const char SEPARATOR = '/';
  constexpr static const char NUM_PLACEHOLDER = '#';
};


inline StringTreeLeaf::StringTreeLeaf(): nodePtr(nullptr)
{  }

}

#endif // ROSTYPE_H
