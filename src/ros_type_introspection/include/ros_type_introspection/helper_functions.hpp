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

#ifndef ROS_INTROSPECTION_HELPER_H
#define ROS_INTROSPECTION_HELPER_H

#include <functional>
#include "ros/ros.h"

namespace RosIntrospection {

// helper function to deserialize raw memory
template <typename T>
inline void ReadFromBuffer(const std::vector<uint8_t>& buffer, size_t& offset, T& destination)
{
    if (offset + sizeof(T) > buffer.size()) {
        throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
    }
    destination = (*(reinterpret_cast<const T*>(&(buffer.data()[offset]))));  //拿出值，如果是string则是字符个数
    offset += sizeof(T);
}

template <>
inline void ReadFromBuffer(const std::vector<uint8_t>& buffer, size_t& offset, std::string& destination)
{
    uint32_t string_size = 0;
    ReadFromBuffer(buffer, offset, string_size);
    if (offset + string_size > buffer.size()) {
        throw std::runtime_error("Buffer overrun in RosIntrospection::ReadFromBuffer");
    }
    const char* buffer_ptr = reinterpret_cast<const char*>(&buffer[offset]);
    offset += string_size;
    destination.assign(buffer_ptr, string_size);
}

template <typename T>
inline std::string ReadFromBufferToVariant(const std::vector<uint8_t>& buffer, size_t& offset)
{
    T destination;
    ReadFromBuffer(buffer, offset, destination);
    return std::to_string(destination);
}
inline std::string ReadFromBufferToVariant(BuiltinType id, const std::vector<uint8_t>& buffer, size_t& offset)
{
    switch (id) {
        case BOOL:
            return ReadFromBufferToVariant<bool>(buffer, offset);
        case CHAR:
            return ReadFromBufferToVariant<char>(buffer, offset);
        case BYTE:
        case UINT8:
            return ReadFromBufferToVariant<uint8_t>(buffer, offset);
        case UINT16:
            return ReadFromBufferToVariant<uint16_t>(buffer, offset);
        case UINT32:
            return ReadFromBufferToVariant<uint32_t>(buffer, offset);
        case UINT64:
            return ReadFromBufferToVariant<uint64_t>(buffer, offset);

        case INT8:
            return ReadFromBufferToVariant<int8_t>(buffer, offset);
        case INT16:
            return ReadFromBufferToVariant<int16_t>(buffer, offset);
        case INT32:
            return ReadFromBufferToVariant<int32_t>(buffer, offset);
        case INT64:
            return ReadFromBufferToVariant<int64_t>(buffer, offset);

        case FLOAT32:
            return ReadFromBufferToVariant<float>(buffer, offset);
        case FLOAT64:
            return ReadFromBufferToVariant<double>(buffer, offset);

        case TIME: {
            ros::Time tmp;
            ReadFromBuffer(buffer, offset, tmp.sec);
            ReadFromBuffer(buffer, offset, tmp.nsec);
            std::string result;
            result = "sec:" + std::to_string(tmp.sec) + "  nsec:" + std::to_string(tmp.nsec);
            return result;
        }
        case DURATION: {
            ros::Duration tmp;
            ReadFromBuffer(buffer, offset, tmp.sec);
            ReadFromBuffer(buffer, offset, tmp.nsec);
            std::string result;
            result = "sec:" + std::to_string(tmp.sec) + "  nsec:" + std::to_string(tmp.nsec);
            return result;
        }

        case STRING: {
            uint32_t string_size = 0;
            ReadFromBuffer(buffer, offset, string_size);
            if (offset + string_size > buffer.size()) {
                throw std::runtime_error("Buffer overrun");
            }
            // Variant var_string(reinterpret_cast<const char*>( &buffer[offset] ), string_size  );
            // offset += string_size;
            return "superMan0";
        }
        case OTHER:
            return "-1";
        default:
            break;
    }
    throw std::runtime_error("unsupported builtin type value");
}

}  // namespace RosIntrospection

#endif
