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


#include "ros_type_introspection/ros_type.hpp"
#include "ros_type_introspection/helper_functions.hpp"

namespace RosIntrospection{

ROSType::ROSType(std::string name):
  baseName_(name)
{
  int pos = -1;
  for (size_t i=0; i<name.size(); i++)
  {
    if(name[i] == '/'){
      pos = i;
      break;
    }
  }

  if( pos == -1)
  {
    msgName_ = baseName_;
  }
  else{
    pkgName_ = std::string( baseName_.data(), pos);
    pos++;
    msgName_ = std::string( baseName_.data() + pos, baseName_.size() - pos);
  }

  id_   = ToBuiltinType( msgName_ );
  hash_ = std::hash<std::string>{}( baseName_ );
}

ROSType& ROSType::operator= (const ROSType &other)
{
    int pos = other.pkgName_.size();
    baseName_ = other.baseName_;
    pkgName_ = std::string( baseName_.data(), pos);
    if( pos > 0) pos++;
    msgName_ = std::string( baseName_.data() + pos, baseName_.size() - pos);
    id_   = other.id_;
    hash_ = other.hash_;
    return *this;
}

ROSType& ROSType::operator= (ROSType &&other)
{
    int pos = other.pkgName_.size();
    baseName_ = std::move( other.baseName_ );
    pkgName_ = std::string( baseName_.data(), pos);
    if( pos > 0) pos++;
    msgName_ = std::string( baseName_.data() + pos, baseName_.size() - pos);
    id_   = other.id_;
    hash_ = other.hash_;
    return *this;
}


void ROSType::SetPkgName(std::string new_pkg)
{
  assert(pkgName_.size() == 0);

  int pos = new_pkg.size();
  baseName_  = new_pkg + "/" + baseName_;
  //baseName_ = absl::Substitute("$0/$1", new_pkg, baseName_);

  pkgName_ = std::string( baseName_.data(), pos++);
  msgName_ = std::string( baseName_.data() + pos, baseName_.size() - pos);

  hash_ = std::hash<std::string>{}( baseName_ );
}


}
