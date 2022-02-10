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

#include "ros_type_introspection/ros_field.hpp"
#include <regex>
namespace RosIntrospection {

ROSField::ROSField(const std::string& definition) : arraySize_(1)
{
    static const std::regex type_regex("[a-zA-Z][a-zA-Z0-9_]*"
                                       "(/[a-zA-Z][a-zA-Z0-9_]*){0,1}"
                                       "(\\[[0-9]*\\]){0,1}");

    static const std::regex field_regex("[a-zA-Z][a-zA-Z0-9_]*");

    static const std::regex array_regex("(.+)(\\[([0-9]*)\\])");

    using std::regex;
    std::string::const_iterator begin = definition.begin();
    std::string::const_iterator end = definition.end();
    std::match_results<std::string::const_iterator> what;

    // Get type and field
    std::string type, value;

    //-------------------------------
    // Find type, field and array size
    if (regex_search(begin, end, what, type_regex)) {
        type = what[0];
        begin = what[0].second;
    } else {
        throw std::runtime_error("Bad type when parsing field: " + definition);
    }

    if (regex_search(begin, end, what, field_regex)) {
        fieldname_ = what[0];
        begin = what[0].second;
    } else {
        throw std::runtime_error("Bad field when parsing field: " + definition);
    }

    std::string temp_type = type;
    if (regex_search(temp_type, what, array_regex)) {
        type = what[1];

        if (what.size() == 3) {
            arraySize_ = -1;
        } else if (what.size() == 4) {
            std::string size(what[3].first, what[3].second);
            arraySize_ = size.empty() ? -1 : atoi(size.c_str());
        } else {
            throw std::runtime_error("Bad array size when parsing field:  " + definition);
        }
    }

    //-------------------------------
    // Find if Constant or comment

    // Determine next character
    // if '=' -> constant, if '#' -> done, if nothing -> done, otherwise error
    if (regex_search(begin, end, what, std::regex("\\S"))) {
        if (what[0] == "=") {
            begin = what[0].second;
            // Copy constant
            if (type == "string") {
                value.assign(begin, end);
            } else {
                if (regex_search(begin, end, what, std::regex("\\s*#"))) {
                    value.assign(begin, what[0].first);
                } else {
                    value.assign(begin, end);
                }
                // TODO: Raise error if string is not numeric
            }
            if (!value.empty()) {
                value.erase(0, value.find_first_not_of(" "));
                value.erase(value.find_last_not_of(" ") + 1);
            }
        } else if (what[0] == "#") {
            // Ignore comment
        } else {
            // Error
            throw std::runtime_error("Unexpected character after type and field:  " + definition);
        }
    }
    type_ = ROSType(type);
    value_ = value;
}

}  // namespace RosIntrospection
