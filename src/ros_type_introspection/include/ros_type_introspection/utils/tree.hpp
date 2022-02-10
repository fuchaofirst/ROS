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


#ifndef STRINGTREE_H
#define STRINGTREE_H

#include <vector>
#include <deque>
#include <iostream>
#include <memory>


namespace RosIntrospection {

namespace details{

template <typename T> class TreeNode
{
public:
  typedef std::vector<TreeNode> childrenVector; // dangerous because of pointer invalidation (but faster)
  TreeNode(const TreeNode* parent );
  const TreeNode* Parent() const  { return parent_; }
  const T& Value()  const          { return value_; }
  void SetValue( const T& value)  { value_ = value; }
  const childrenVector& Children()const   { return children_; }
  childrenVector& Children()              { return children_; }
  const TreeNode* Child(size_t index) const { return &(children_[index]); }
  TreeNode* Child(size_t index) { return &(children_[index]); }
  TreeNode *AddChild(const T& child );
  bool IsLeaf() const { return children_.empty(); }
private:
  const TreeNode*   parent_;
  T                 value_;
  childrenVector    children_;
};


template <typename T> class Tree
{
public:
  Tree(): _root( new TreeNode<T>(nullptr) ) {}
  const TreeNode<T>* croot() const { return _root.get(); }
  TreeNode<T>* root() { return _root.get(); }
private:
  std::unique_ptr<TreeNode<T>> _root;
};


template <typename T> inline
TreeNode<T>::TreeNode(const TreeNode *parent):
  parent_(parent)
{

}

template <typename T> inline
TreeNode<T> *TreeNode<T>::AddChild(const T& value)
{
 // assert(children_.capacity() > children_.size() );
  children_.push_back( TreeNode<T>(this) );
  children_.back().SetValue( value );
  return &children_.back();
}

}

}

#endif 
