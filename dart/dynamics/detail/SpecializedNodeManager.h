/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_H_
#define DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_H_

#include "dart/dynamics/SpecializedNodeManager.h"

namespace dart {
namespace dynamics {

// This preprocessor token should only be used by the unittest that is
// responsible for checking that the specialized routines are being used to
// access specialized Addons
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
bool usedSpecializedNodeAccess;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

//==============================================================================
template <class SpecNode>
SpecializedNodeManagerForBodyNode<SpecNode>::SpecializedNodeManagerForBodyNode()
{
  mNodeMap[typeid( SpecNode )] = std::vector<Node*>();
  mSpecNodeIterator = mNodeMap.find(typeid( SpecNode ));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
size_t SpecializedNodeManagerForBodyNode<SpecNode>::getNumNodes() const
{
  return _getNumNodes(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForBodyNode<SpecNode>::getNode(size_t index)
{
  return _getNode(type<NodeType>(), index);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* SpecializedNodeManagerForBodyNode<SpecNode>::getNode(size_t index) const
{
  return const_cast<SpecializedNodeManagerForBodyNode<SpecNode>*>(this)->
      _getNode(type<NodeType>(), index);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SpecializedNodeManagerForBodyNode<SpecNode>::isSpecializedForNode()
{
  return _isSpecializedForNode(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
size_t SpecializedNodeManagerForBodyNode<SpecNode>::_getNumNodes(type<NodeType>) const
{
  return detail::BasicNodeManagerForBodyNode::getNumNodes<NodeType>();
}

//==============================================================================
template <class SpecNode>
size_t SpecializedNodeManagerForBodyNode<SpecNode>::_getNumNodes(type<SpecNode>) const
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return mSpecNodeIterator->second.size();
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForBodyNode<SpecNode>::_getNode(type<NodeType>, size_t index)
{
  return detail::BasicNodeManagerForBodyNode::getNode<NodeType>(index);
}

//==============================================================================
template <class SpecNode>
SpecNode* SpecializedNodeManagerForBodyNode<SpecNode>::_getNode(type<SpecNode>, size_t index)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return static_cast<SpecNode*>(
        getVectorObjectIfAvailable(index, mSpecNodeIterator->second));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SpecializedNodeManagerForBodyNode<SpecNode>::_isSpecializedForNode(type<NodeType>)
{
  return false;
}

//==============================================================================
template <class SpecNode>
constexpr bool SpecializedNodeManagerForBodyNode<SpecNode>::_isSpecializedForNode(type<SpecNode>)
{
  return true;
}

//==============================================================================
template <class SpecNode>
SpecializedNodeManagerForSkeleton<SpecNode>::SpecializedNodeManagerForSkeleton()
{
  mSpecializedTreeNodes[typeid( SpecNode )] = &mTreeSpecNodeIterators;

  mNodeNameMgrMap[typeid( SpecNode )] = common::NameManager<Node*>();
  mSpecNodeNameMgrIterator = mNodeNameMgrMap.find(typeid( SpecNode ));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
size_t SpecializedNodeManagerForSkeleton<SpecNode>::getNumNodes(
    size_t treeIndex) const
{
  return _getNumNodes(type<NodeType>(), treeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::getNode(
    size_t treeIndex, size_t nodeIndex)
{
  return _getNode(type<NodeType>(), treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::getNode(
    size_t treeIndex, size_t nodeIndex) const
{
  return const_cast<SpecializedNodeManagerForSkeleton<SpecNode>*>(this)->
        _getNode(type<NodeType>(), treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::getNode(
    const std::string& name)
{
  return _getNode(type<NodeType>(), name);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
const NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::getNode(
    const std::string& name) const
{
  return const_cast<SpecializedNodeManagerForSkeleton<SpecNode>*>(this)->
        _getNode(type<NodeType>(), name);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SpecializedNodeManagerForSkeleton<SpecNode>::
    isSpecializedForNode()
{
  return _isSpecializedForNode(type<NodeType>());
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
size_t SpecializedNodeManagerForSkeleton<SpecNode>::_getNumNodes(
    type<NodeType>, size_t treeIndex) const
{
  return detail::BasicNodeManagerForSkeleton::getNumNodes<NodeType>(treeIndex);
}

//==============================================================================
template <class SpecNode>
size_t SpecializedNodeManagerForSkeleton<SpecNode>::_getNumNodes(
    type<SpecNode>, size_t treeIndex) const
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNumNodes<" << typeid(SpecNode).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return 0;
  }

  return mTreeSpecNodeIterators[treeIndex]->second.size();
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::_getNode(
    type<NodeType>, size_t treeIndex, size_t nodeIndex)
{
  return detail::BasicNodeManagerForSkeleton::getNode<NodeType>(
        treeIndex, nodeIndex);
}

//==============================================================================
template <class SpecNode>
SpecNode* SpecializedNodeManagerForSkeleton<SpecNode>::_getNode(
    type<SpecNode>, size_t treeIndex, size_t nodeIndex)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(SpecNode).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return nullptr;
  }

  NodeMap::iterator& it = mTreeSpecNodeIterators[treeIndex];

  if(nodeIndex >= it->second.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(SpecNode).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are only (" << it->second.size() << ") Nodes of the "
          << "requested type within that tree\n";
    assert(false);
    return nullptr;
  }

  return static_cast<SpecNode*>(it->second[nodeIndex]);
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
NodeType* SpecializedNodeManagerForSkeleton<SpecNode>::_getNode(
    type<NodeType>, const std::string& name)
{
  return detail::BasicNodeManagerForSkeleton::getNode<NodeType>(name);
}

//==============================================================================
template <class SpecNode>
SpecNode* SpecializedNodeManagerForSkeleton<SpecNode>::_getNode(
    type<SpecNode>, const std::string& name)
{
#ifdef DART_UNITTEST_SPECIALIZED_NODE_ACCESS
  usedSpecializedNodeAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_NODE_ACCESS

  return static_cast<SpecNode*>(mSpecNodeNameMgrIterator->second.getObject(name));
}

//==============================================================================
template <class SpecNode>
template <class NodeType>
constexpr bool SpecializedNodeManagerForSkeleton<SpecNode>::
    _isSpecializedForNode(type<NodeType>)
{
  return false;
}

//==============================================================================
template <class SpecNode>
constexpr bool SpecializedNodeManagerForSkeleton<SpecNode>::
    _isSpecializedForNode(type<SpecNode>)
{
  return true;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SPECIALIZEDNODEMANAGER_H_
