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

#include <regex>
#include <string>
#include <functional>
#include "ros_type_introspection/ros_introspection.hpp"
#include "ros_type_introspection/helper_functions.hpp"
#include "ros_type_introspection/utils/tree.hpp"
#include "ros_type_introspection/ros_message.hpp"
#include <unordered_map>
#include <vector>
#include <algorithm>
void SelfSplit(std::vector<std::string>& output, const std::string& inString, const std::string& split)
{
    std::string::size_type posStart = 0;
    std::string::size_type posFind = inString.find(split, posStart);
    while (std::string::npos != posFind) {
        if (posStart != posFind) {
            output.push_back(inString.substr(posStart, posFind - posStart));
        }
        posStart = posFind + split.length();
        posFind = inString.find(split, posStart);
    }
    if (posStart != inString.length()) {
        output.push_back(inString.substr(posStart));
    }
}

namespace RosIntrospection {

void Parser::CreateTrees(ROSMessageInfo& info, const std::string& type_name) const
{
    std::function<void(const ROSMessage*, StringTreeNode*, MessageTreeNode*)> recursiveTreeCreator;

    recursiveTreeCreator =
        [&](const ROSMessage* msg_definition, StringTreeNode* string_node, MessageTreeNode* msg_node) {
            // note: should use reserve here, NOT resize
            const size_t NUM_FIELDS = msg_definition->fields().size();

            string_node->Children().reserve(NUM_FIELDS);
            msg_node->Children().reserve(NUM_FIELDS);

            for (const ROSField& field : msg_definition->fields()) {
                if (field.IsConstant() == false) {

                    // Let's add first a child to string_node
                    string_node->AddChild(field.Name());
                    StringTreeNode* new_string_node = &(string_node->Children().back());
                    if (field.IsArray()) {
                        new_string_node->Children().reserve(1);
                        new_string_node = new_string_node->AddChild("#");
                    }

                    const ROSMessage* next_msg = nullptr;
                    // builtin types will not trigger a recursion
                    if (field.Type().IsBuiltin() == false) {
                        next_msg = GetMessageByType(field.Type(), info);
                        if (next_msg == nullptr) {
                            throw std::runtime_error("This type was not registered ");
                        }
                        msg_node->AddChild(next_msg);
                        MessageTreeNode* new_msg_node = &(msg_node->Children().back());
                        recursiveTreeCreator(next_msg, new_string_node, new_msg_node);
                    }
                }  // end of field.IsConstant()
            }      // end of for fields
        };         // end of lambda

    info.stringtree_.root()->SetValue(type_name);
    info.messagetree_.root()->SetValue(&info.typelist_.front());
    recursiveTreeCreator(&info.typelist_.front(), info.stringtree_.root(), info.messagetree_.root());
}

inline bool operator==(const std::string& a, const std::string& b)
{
    return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
}

void Parser::RegisterMessageDefinition(const std::string& msg_definition,  // xtopic名字
    const ROSType& main_type,                                              //消息类型
    const std::string& definition)                                         //消息具体定义
{

    if (registeredMessages.count(msg_definition) > 0) {
        return;  // already registered
    }
    const std::regex msg_separation_regex("\\n=+\\n+");

    std::vector<std::string> split;
    std::vector<const ROSType*> all_types;

    std::string uniqueString = "!(+&^%$@*2019815\n";
    std::string definition2 = std::regex_replace(definition, msg_separation_regex, uniqueString);
    SelfSplit(split, definition2, uniqueString);
    ROSMessageInfo info;
    info.typelist_.reserve(split.size());

    for (size_t i = 0; i < split.size(); ++i) {
        ROSMessage msg(split[i]);
        if (i == 0) {
            msg.mutateType(main_type);
        }

        info.typelist_.push_back(std::move(msg));
        all_types.push_back(&(info.typelist_.back().Type()));
    }

    for (ROSMessage& msg : info.typelist_) {
        msg.updateMissingPkgNames(all_types);
    }
    CreateTrees(info, msg_definition);
    registeredMessages.insert(std::make_pair(msg_definition, std::move(info)));
}

const ROSMessageInfo* Parser::GetMessageInfo(const std::string& msgIdentifier) const
{
    auto it = registeredMessages.find(msgIdentifier);
    if (it != registeredMessages.end()) {
        return &(it->second);
    }
    return nullptr;
}

const ROSMessage* Parser::GetMessageByType(const ROSType& type, const ROSMessageInfo& info) const
{
    for (const ROSMessage& msg : info.typelist_)  // find in the list
    {
        if (msg.Type() == type) {
            return &msg;
        }
    }
    return nullptr;
}

/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
uint32_t Parser::TreeRecrus(StringTreeNode* node) const
{
    if (!node->Children().size()) {
        return 0;
    }
    for (auto temp : node->Children()) {
        std::cout << temp.Value() << std::endl;
        TreeRecrus(&temp);
    }
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
std::string Parser::GetBlankStr(uint32_t idx) const
{
    std::string result;
    for (uint32_t id = 0; id < idx; id++) {
        result = result + "  ";
    }
    return result;
}
/**
 * @brief: split the string by useing of the regex pattern
 * @param [in]  : strintg of inputing, regex exp.
 * @param [out] : the result after spliting
 * @return      :
 */
std::string Parser::SplitHeader(std::string& str, std::string pattern, uint32_t& nums) const
{
    std::string::size_type pos;
    std::string result;
    std::string input = str;
    input += pattern;
    uint32_t size = input.size();
    uint32_t blankNum = 0;
    for (uint32_t i = 0; i < size; i++) {
        pos = input.find(pattern, i);
        if (pos < size) {
            std::string s = input.substr(i, pos - i);
            if (s != "") {
                result = result + GetBlankStr(blankNum) + s + ":\n";
                blankNum++;
            }
            i = pos + pattern.size() - 1;
        }
    }
    nums = blankNum;
    return result;
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
bool Parser::DividFullName(std::string& fullName, std::string& headerStr, std::string& varNameStr) const
{
    uint32_t varFullNameLen = fullName.length();
    uint32_t findSyms = varFullNameLen - 1;
    uint32_t headerEnd = 0;
    if (findSyms > varFullNameLen) {
        return false;
    }
    if (fullName[findSyms] == '/') {
        findSyms = findSyms - 1;
    }
    if (findSyms > varFullNameLen) {
        return false;
    }
    while (fullName[findSyms] != '/') {
        findSyms--;
    }
    headerEnd = findSyms + 1;

    if (headerEnd >= varFullNameLen) {
        return false;
    }
    headerStr = fullName.substr(0, headerEnd);
    varNameStr = fullName.substr(headerEnd, varFullNameLen - headerEnd);
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
std::string Parser::StyleMsg(std::map<std::string, std::string>& arryData, std::vector<std::string>& printInfo,
    styleMsgParmt& styleMsgParm) const
{
    if (styleMsgParm.varFullName[styleMsgParm.varFullName.length() - 1] == '/') {
        styleMsgParm.arrydata = styleMsgParm.arrydata + " " + styleMsgParm.value + " ";
        arryData[styleMsgParm.varFullName] = styleMsgParm.arrydata;
    } else {
        printInfo.push_back(styleMsgParm.varFullName + ": " + styleMsgParm.value);
    }
    return "ok";
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :GetFullName
 */
std::string Parser::GetFullName(StringTreeNode* parseTree, std::string value) const
{
    uint32_t idd = 0;
    std::string varFullName = parseTree->Value();
    if (varFullName == "#") {
        varFullName = "";
    }
    parseTree = (StringTreeNode*)parseTree->Parent();
    while (parseTree != nullptr) {  //获取根节点
         std::string testvalue = parseTree->Value();
        if (testvalue == "#") {
            parseTree = (StringTreeNode*)parseTree->Parent();
            continue;
        }
        varFullName = testvalue + "/" + varFullName;
        parseTree = (StringTreeNode*)parseTree->Parent();
        idd++;
        if (idd > 6) {
            std::cout << "error" << value << std::endl;
            break;
        }
    }
    return varFullName;
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
bool Parser::DisposMsgStyle(
    const FlatMessage* flatContainer, uint32_t nameIndex, uint32_t valueIndex, bool nameOrindex) const
{

    std::map<std::string, std::vector<std::string>> msgDataBase;
    std::map<std::string, std::string> arryData;
    std::vector<std::string> orderArry;
    uint32_t allNum = (nameOrindex == true ? valueIndex : nameIndex);
    bool start = true;
    styleMsgParmt styleMsgParm;
    std::vector<std::string> printInfo;
    bool lastIsArryData = false;
    uint32_t count = 0;
    std::map<std::string, bool> bigDataParsCnt;
    for (uint32_t idx = 0; idx < allNum; idx++) {
        StringTreeNode* parseTree = nullptr;
        std::string value;
        if (nameOrindex == false) {
            parseTree = (StringTreeNode*)flatContainer->name[idx].first.nodePtr;
            value = flatContainer->name[idx].second;
        } else {
            parseTree = (StringTreeNode*)flatContainer->value[idx].first.nodePtr;
            value = flatContainer->value[idx].second;
        }
        if(parseTree==nullptr){
            continue;
        }
        
        styleMsgParm.varFullName = GetFullName(parseTree, value);

        std::string headerStr;
        std::string varNameStr;
        DividFullName(styleMsgParm.varFullName, headerStr, varNameStr);

        std::vector<std::string>::iterator it = find(orderArry.begin(), orderArry.end(), headerStr);
        if (it == orderArry.end()) {
            orderArry.push_back(headerStr);
        }
        //待完成 如果总变量数超80 不打印
        if (msgDataBase[styleMsgParm.varFullName].size() > 80) {
            continue;
        }
        std::string begin = "";
        if (lastIsArryData) {
            begin = "\n";
            lastIsArryData = false;
        }
        if (styleMsgParm.varFullName[styleMsgParm.varFullName.size() - 1] != '/') {
            std::cout << begin << styleMsgParm.varFullName << ": " << value << std::endl;
            lastIsArryData = false;
        } else {
            lastIsArryData = true;
            if (!bigDataParsCnt[styleMsgParm.varFullName]) {
                std::cout << begin << styleMsgParm.varFullName << ": " << std::endl;
                bigDataParsCnt[styleMsgParm.varFullName] = true;
            }
            std::cout << " " << value << " ";
        }
    }
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
bool Parser::IsBlob(FlatMessage* flatContainer, DeserlizeParas& deserlizeParas, std::vector<uint8_t>& buffer,
    int32_t& array_size, const StringTreeLeaf& newTreeLeaf) const
{
    if (flatContainer->blob.size() <= deserlizeParas.blob_index) {
        const size_t increased_size = std::max(size_t(32), flatContainer->blob.size() * 3 / 2);
        flatContainer->blob.resize(increased_size);
    }
    if (deserlizeParas.buffer_offset + array_size > buffer.size()) {
        throw std::runtime_error("Buffer overrun in DeserializeIntoFlatContainer (blob)");
    }
    bool bigArrySkip = false;

    if (deserlizeParas.bigArrySize && (!bigArrySkip)) {
        flatContainer->blob[deserlizeParas.blob_index].first = newTreeLeaf;
        std::vector<uint8_t>& blob = flatContainer->blob[deserlizeParas.blob_index].second;
        deserlizeParas.blob_index++;
        blob.resize(array_size);
        std::memcpy(blob.data(), &buffer[deserlizeParas.buffer_offset], array_size);
    }
    deserlizeParas.buffer_offset += array_size;
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
bool Parser::StringDeserlize(FlatMessage* flatContainer, DeserlizeParas& deserlizeParas, std::vector<uint8_t>& buffer,
    const StringTreeLeaf& newTreeLeaf) const
{

    if (flatContainer->name.size() <= deserlizeParas.name_index) {
        const size_t increased_size = std::max(size_t(32), flatContainer->name.size() * 3 / 2);
        flatContainer->name.resize(increased_size);
    }
    std::string& name =
        flatContainer->name[deserlizeParas.name_index].second;  // read directly inside an existing std::string

    ReadFromBuffer<std::string>(buffer, deserlizeParas.buffer_offset, name);
    if (deserlizeParas.bigArrySize) {
        flatContainer->name[deserlizeParas.name_index].first = newTreeLeaf;
        deserlizeParas.name_index++;
    }
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
std::pair<bool, bool> Parser::ArryOrBlob(arryBlobParat& arryBlobPara, StringTreeLeaf& newTreeLeaf,
    DeserlizeParas& deserlizeParas, int32_t& array_size, std::vector<uint8_t>& buffer) const
{

    bool IS_BLOB = false;
    bool DO_STORE = deserlizeParas.bigArrySize;
    if (array_size == -1) {
        ReadFromBuffer(buffer, deserlizeParas.buffer_offset, array_size);
    }
    if (arryBlobPara.isArry) {
        newTreeLeaf.indexarray.push_back(0);
        newTreeLeaf.nodePtr = newTreeLeaf.nodePtr->Child(0);
    }
    // Stop storing it if is NOT a blob and a very large array.
    if (array_size > static_cast<int32_t>(deserlizeParas.max_array_size)) {
        if (arryBlobPara.builtinSize == 1) {  // 单字节
            IS_BLOB = false;
        }
        // } else {          // debug
        //     if (discardLargeArray_) {
        //         DO_STORE = false;
        //     }
        //     deserlizeParas.entireMessageParse = false;
        // }
    }
    std::pair<bool, bool> result;
    result.first = IS_BLOB;
    result.second = DO_STORE;
    return result;
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
void Parser::SampleTypeDeserlize(std::pair<bool, BuiltinType>& pars, FlatMessage* flatContainer,
    DeserlizeParas& deserlizeParas, std::vector<uint8_t>& buffer, StringTreeLeaf& newTreeLeaf) const
{
    if (flatContainer->value.size() <= deserlizeParas.value_index) {
        const size_t increased_size = std::max(size_t(32), flatContainer->value.size() * 3 / 2);
        flatContainer->value.resize(increased_size);
    }
    std::string var = ReadFromBufferToVariant(pars.second, buffer, deserlizeParas.buffer_offset);
    if (pars.first) {
        flatContainer->value[deserlizeParas.value_index] =
            std::make_pair(newTreeLeaf, var);  //保存的值，如果是数组，保存数组元素
        deserlizeParas.value_index++;
    }
}
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */
bool Parser::DeserializeImpl(const MessageTreeNode* msg_node, const StringTreeLeaf& treeLeaf,
    std::vector<uint8_t>& buffer, FlatMessage* flatContainer, DeserlizeParas& deserlizeParas) const
{
    bool DO_STORE = deserlizeParas.bigArrySize;
    const ROSMessage* msg_definition = msg_node->Value();
    size_t index_s = 0;
    size_t index_m = 0;
    for (const ROSField& field : msg_definition->fields()) {
        if (field.IsConstant()) {
            continue;
        }
        arryBlobParat arryBlobPara;
        auto newTreeLeaf = treeLeaf;
        int32_t array_size = field.ArraySize();
        const ROSType& fieldType = field.Type();
        newTreeLeaf.nodePtr = treeLeaf.nodePtr->Child(index_s);
        arryBlobPara.isArry = field.IsArray();
        arryBlobPara.builtinSize = BuiltinSize(fieldType.typeID());
        std::pair<bool, bool> result = ArryOrBlob(arryBlobPara, newTreeLeaf, deserlizeParas, array_size, buffer);
        DO_STORE = result.second;
        if (result.first)  // special case. This is a "blob", typically an image, a map, etc.
        {
            deserlizeParas.bigArrySize = DO_STORE;  // blob型数据暂时不处理
            IsBlob(flatContainer, deserlizeParas, buffer, array_size, newTreeLeaf);
            std::cout << "[WARN]:arry data is so big ,will not be presented!\n" << std::endl;
        } else  // NOT a BLOB
        {
            bool DO_STORE_ARRAY = DO_STORE;
            for (int i = 0; i < array_size; i++) {
                if (DO_STORE_ARRAY && i >= deserlizeParas.max_array_size) {
                    DO_STORE_ARRAY = false;
                }
                if (field.IsArray() && DO_STORE_ARRAY) {
                    newTreeLeaf.indexarray.back() = i;
                }
                if (fieldType.typeID() == STRING) {
                    deserlizeParas.bigArrySize = DO_STORE_ARRAY;
                    StringDeserlize(flatContainer, deserlizeParas, buffer, newTreeLeaf);
                } else if (fieldType.IsBuiltin()) {
                    std::pair<bool, BuiltinType> pars(DO_STORE_ARRAY, fieldType.typeID());
                    SampleTypeDeserlize(pars, flatContainer, deserlizeParas, buffer, newTreeLeaf);
                } else {  // fieldType.typeID() == OTHER
                    DeserializeImpl(msg_node->Child(index_m), newTreeLeaf, buffer, flatContainer, deserlizeParas);
                }
            }  // end for array_size
        }
        if (fieldType.typeID() == OTHER) {
            index_m++;
        }
        index_s++;
    }  // end for fields
};
/**
 * @brief      :
 * @param [in] :
 * @param [out]:
 * @return     :
 */

bool Parser::DeserializeIntoFlatContainer(const std::string& msgIdentifier, std::vector<uint8_t> buffer,
    FlatMessage* flatContainer, const uint32_t max_array_size) const
{

    const ROSMessageInfo* msg_info = GetMessageInfo(msgIdentifier);  //之前已经根据topicname进行了注册

    if (msg_info == nullptr) {
        throw std::runtime_error(
            "DeserializeIntoFlatContainer: msgIdentifier not registerd. Use registerMessageDefinition");
    }

    flatContainer->tree = &msg_info->stringtree_;
    StringTreeLeaf rootnode;
    rootnode.nodePtr = msg_info->stringtree_.croot();

    DeserlizeParas deserlizeParas;
    deserlizeParas.max_array_size = max_array_size;
    deserlizeParas.entireMessageParse = true;
    DeserializeImpl(msg_info->messagetree_.croot(), rootnode, buffer, flatContainer, deserlizeParas);

    flatContainer->name.resize(deserlizeParas.name_index);
    flatContainer->value.resize(deserlizeParas.value_index);
    bool bigArrySkip = false;
    if (!bigArrySkip) {
        flatContainer->blob.resize(deserlizeParas.blob_index);
    }
    deserlizeParas.buffer_offset = buffer.size();
    if (deserlizeParas.buffer_offset != buffer.size()) {
        throw std::runtime_error("buildRosFlatType: There was an error parsing the buffer");
    }

    DisposMsgStyle(flatContainer, deserlizeParas.name_index, deserlizeParas.value_index, true);
    // std::cout << "----error-------ok-----------------" << std::endl;
    DisposMsgStyle(flatContainer, deserlizeParas.name_index, deserlizeParas.value_index, false);
    return deserlizeParas.entireMessageParse;
}

}  // namespace RosIntrospection
