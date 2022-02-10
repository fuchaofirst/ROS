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

#ifndef ROS_INTROSPECTION_HPP
#define ROS_INTROSPECTION_HPP
#include <unordered_map>
#include <unordered_set>
#include "ros_type_introspection/stringtree_leaf.hpp"
#include "ros_type_introspection/helper_functions.hpp"
#include "ros_field.hpp"
#include "ros_type.hpp"
namespace RosIntrospection {

struct FlatMessage {

    const StringTree* tree;

    std::vector<std::pair<StringTreeLeaf, std::string>> value;

    std::vector<std::pair<StringTreeLeaf, std::string>> name;

    std::vector<std::pair<StringTreeLeaf, std::vector<uint8_t>>> blob;
};

typedef struct DeserlizePara {
    uint32_t max_array_size;
    size_t buffer_offset;
    int32_t array_size;
    size_t value_index;
    size_t name_index;
    size_t blob_index;

    bool bigArrySize;
    bool entireMessageParse;

    DeserlizePara()
    {
        buffer_offset = 0;
        value_index = 0;
        name_index = 0;
        blob_index = 0;
        bigArrySize = true;
        entireMessageParse = true;
    }

} DeserlizeParas;

typedef struct blobPara {
    int typeID;
    bool isBlob;
    bool doStore;

    blobPara()
    {
        isBlob = 0;
        doStore = 0;
        typeID = 0;
    }
} blobParas;

typedef struct arryBlobParas
{
    int32_t array_size;
    int builtinSize;
    bool isArry;
    arryBlobParas()
    {
        array_size = 0;
        builtinSize = 0;
        isArry = 0;
    }
}arryBlobParat;

typedef struct styleMsgParms {
    std::string varFullName;
    std::string lastFullName;
    std::string arrydata;
    std::string value;
} styleMsgParmt;

class Parser {

public:
    Parser() : globalWarnings_(&std::cerr)
    {}

    void RegisterMessageDefinition(
        const std::string& message_identifier, const ROSType& main_type, const std::string& definition);

    const ROSMessageInfo* GetMessageInfo(const std::string& msgIdentifier) const;

    const ROSMessage* GetMessageByType(const ROSType& type, const ROSMessageInfo& info) const;

    bool DeserializeIntoFlatContainer(const std::string& msgIdentifier, std::vector<uint8_t> buffer,
        FlatMessage* flatContainer, const uint32_t max_array_size) const;

    void setWarningsStream(std::ostream* output)
    {
        globalWarnings_ = output;
    }
    uint32_t TreeRecrus(StringTreeNode* node) const;
    bool DisposMsgStyle(
        const FlatMessage* flatContainer, uint32_t nameIndex, uint32_t valueIndex, bool nameOrindex) const;
    std::string StyleMsg(std::map<std::string, std::string>& arryData,  std::vector<std::string>& printInfo, styleMsgParmt& styleMsgParm) const;
    bool DividFullName(std::string& fullName, std::string& headerStr, std::string& varNameStr) const;
    std::string SplitHeader(std::string& str, std::string pattern, uint32_t& nums) const;
    std::string GetBlankStr(uint32_t idx) const;
    void CreateTrees(ROSMessageInfo& info, const std::string& type_name) const;
    bool DeserializeImpl(const MessageTreeNode* msg_node, const StringTreeLeaf& treeLeaf, std::vector<uint8_t>& buffer,
        FlatMessage* flatContainer, DeserlizeParas& deserlizeParas) const;

    bool IsNotBlob(const MessageTreeNode* msg_node, FlatMessage* flatContainer, DeserlizeParas& deserlizeParas,
        std::vector<uint8_t>& buffer, blobParas& blobParat) const;
    bool IsBlob(FlatMessage* flatContainer, DeserlizeParas& deserlizeParas, std::vector<uint8_t>& buffer,
        int32_t& array_size, const StringTreeLeaf& newTreeLeaf) const;
    bool StringDeserlize(FlatMessage* flatContainer, DeserlizeParas& deserlizeParas, std::vector<uint8_t>& buffer,
        const StringTreeLeaf& newTreeLeaf) const;
    void UpDateBLobInfo(int fieldTypeId, bool& isBlob, bool& doStore, bool isArry, int32_t& array_size,
        StringTreeLeaf& newTreeLeaf, DeserlizeParas& deserlizeParas) const;
    std::pair<bool, bool> ArryOrBlob(arryBlobParat &arryBlobPara,StringTreeLeaf& newTreeLeaf,
    DeserlizeParas& deserlizeParas, int32_t& array_size, std::vector<uint8_t>& buffer)const;
    void SampleTypeDeserlize(std::pair<bool, BuiltinType>& pars, FlatMessage* flatContainer, DeserlizeParas& deserlizeParas,
    std::vector<uint8_t>& buffer, StringTreeLeaf& newTreeLeaf)const;
    std::string GetFullName(StringTreeNode *parseTree,std::string value) const;

private:
    std::unordered_map<std::string, ROSMessageInfo> registeredMessages;

    std::ostream* globalWarnings_;
    bool discardLargeArray_;
};
}  // namespace RosIntrospection

#endif  // ROS_INTROSPECTION_HPP
