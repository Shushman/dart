#ifndef DART_UTILS_KINBODY_PARSER_H
#define DART_UTILS_KINBODY_PARSER_H

#include <cstddef>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/common/Deprecated.h"
#include "dart/utils/Parser.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/SingleDofJoint.h"
#include "dart/dynamics/MultiDofJoint.h"
#include "dart/simulation/World.h"

//CURRENTLY ASSUMES ONLY ONE BODY NODE

namespace dart {

namespace dynamics {
class BodyNode;
class Shape;
class Joint;
class Skeleton;
}

namespace utils{

class SkelParser
{
public:

  static dynamics::SkeletonPtr readKinBodyXMLFile(
    const common::Uri& _fileUri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);

  typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;

  struct SkelBodyNode
  {
    BodyPropPtr properties;
    Eigen::Isometry3d initTransform;
    std::string type;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

protected:

  static dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* _KinBodyElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

  static SkelBodyNode readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

  static dynamics::ShapePtr readShape(
    tinyxml2::XMLElement* _shapeElement,
    const std::string& bodyName,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);


}; //class SkelParser

} //namespace utils

} //namespace dart