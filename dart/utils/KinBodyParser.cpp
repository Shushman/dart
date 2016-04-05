
#include "dart/utils/KinBodyParser.h"
#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "dart/config.h"
#include "dart/common/Console.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/simulation/World.h"
#include "dart/utils/XmlHelpers.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"

namespace dart {

namespace dynamics {
class BodyNode;
class Shape;
class Skeleton;
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class PlanarJoint;
class FreeJoint;
class Marker;
} // namespace dynamics

namespace simulation {
class World;
} // namespace simulation


namespace utils {

namespace {

enum NextResult
{
  VALID,
  CONTINUE,
  BREAK,
  CREATE_FREEJOINT_ROOT
};

using BodyPropPtr = std::shared_ptr<dynamics::BodyNode::Properties>;
using JointPropPtr = std::shared_ptr<dynamics::Joint::Properties>;

struct SkelBodyNode
{
  BodyPropPtr properties;
  Eigen::Isometry3d initTransform;
  std::string type;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SkelJoint
{
  JointPropPtr properties;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd force;
  std::string parentName;
  std::string childName;
  std::string type;
};

// first: BodyNode name | second: BodyNode information
using BodyMap = Eigen::aligned_map<std::string, SkelBodyNode>;

// first: Child BodyNode name | second: Joint information
using JointMap = std::map<std::string, SkelJoint>;

// first: Order that Joint appears in file | second: Child BodyNode name
using IndexToJoint = std::map<size_t, std::string>;

// first: Child BodyNode name | second: Order that Joint appears in file
using JointToIndex = std::map<std::string, size_t>;


//Method defs
dart::dynamics::SkeletonPtr readKinBody(
  tinyxml2::XMLElement* _KinBodyElement,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readBodyNode(
  tinyxml2::XMLElement* _bodyNodeElement,
  const Eigen::Isometry3d& _skeletonFrame,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever);

//Similar to SkelParser::readShape
dynamics::ShapePtr readShape(
  tinyxml2::XMLElement* vizOrColEle,
  const std::string& fieldName, //Either Render or Data for KinBody - may need to make more general
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);


void readAddons(
    const dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* _KinBodyElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

common::ResourceRetrieverPtr getRetriever(
  const common::ResourceRetrieverPtr& _retriever);

} //anonymous namespace


//==============================================================================
dynamics::SkeletonPtr KinBodyParser::readKinBodyXMLFile(
  const common::Uri& _fileUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _kinBodyFile;
  try
  {
    openXMLFile(_kinBodyFile, _fileUri, _retriever);
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile [" << _fileUri.toString() << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  tinyxml2::XMLElement* kinBodyElement = nullptr;
  kinBodyElement = _kinBodyFile.FirstChildElement("KinBody");
  if (kinBodyElement == nullptr)
  {
    dterr << "KinBody  file[" << _fileUri.toString()
          << "] does not contain <KinBody> as the element.\n";
    return nullptr;
  }

  dynamics::SkeletonPtr newSkeleton = readKinBody(
    kinBodyElement, _fileUri, retriever);

  return newSkeleton;
}

namespace {

//==============================================================================
dynamics::SkeletonPtr readKinBody(
  tinyxml2::XMLElement* _KinBodyElement,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{

  assert(_KinBodyElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttributeString(_KinBodyElement, "name");
  newSkeleton->setName(name);

  //Get Body node
  tinyxml2::XMLElement* bodyElement = nullptr;
  bodyElement = _KinBodyElement->FirstChildElement("Body");
  if (bodyElement == nullptr)
  {
    dterr << "KinBody file[" << _baseUri.toString()
          << "] does not contain <Body> element "
          <<"under <KinBody> element.\n";
    return nullptr;
  }

  SkelBodyNode newBodyNode = readBodyNode(
    bodyElement, skeletonFrame,_baseUri, _retriever);

  //How to add newBodyNode to new Skeleton?
  //Also need to add some static joint I guess
  //JS HELP!

  readAddons(newSkeleton, _KinBodyElement, _baseUri, _retriever);
  newSkeleton->resetPositions();
  newSkeleton->resetVelocities();

  return newSkeleton;

}

SkelBodyNode readBodyNode(
  tinyxml2::XMLElement* _bodyNodeElement,
  const Eigen::Isometry3d& _skeletonFrame,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  assert(_bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = getAttributeString(_bodyNodeElement, "name");

  // transformation
  if (hasElement(_bodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_bodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
  }


  //Get SkelBodyNode from PropPtr
  SkelBodyNode skelBodyNode;
  skelBodyNode.properties = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}


//==============================================================================
dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  assert(bodyNode);

  auto shape = readShape(shapeNodeEle, shapeNodeName, baseUri, retriever);
  auto shapeNode = bodyNode->createShapeNode(shape, shapeNodeName);

  // Transformation
  if (hasElement(shapeNodeEle, "transformation"))
  {
    Eigen::Isometry3d W = getValueIsometry3d(shapeNodeEle, "transformation");
    shapeNode->setRelativeTransform(W);
  }

  return shapeNode;
}

//==============================================================================
void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  dynamics::ShapeNode* newShapeNode
      = readShapeNode(bodyNode, vizShapeNodeEle,
                      "Render",
                      baseUri, retriever);

  auto visualAddon = newShapeNode->getVisualAddon(true);

  // color
  if (hasElement(vizShapeNodeEle, "color"))
  {
    Eigen::Vector3d color = getValueVector3d(vizShapeNodeEle, "color");
    visualAddon->setColor(color);
  }
}

//==============================================================================
void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  dynamics::ShapeNode* newShapeNode
      = readShapeNode(bodyNode, collShapeNodeEle,
                      "Data",
                      baseUri, retriever);

  auto collisionAddon = newShapeNode->getCollisionAddon(true);
  newShapeNode->createDynamicsAddon();

  // collidable
  if (hasElement(collShapeNodeEle, "collidable"))
  {
    const bool collidable = getValueDouble(collShapeNodeEle, "collidable");
    collisionAddon->setCollidable(collidable);
  }
}



//Similar to SkelParser::readShape
dynamics::ShapePtr readShape(
  tinyxml2::XMLElement* vizOrColEle,
  const std::string& fieldName, //Either Render or Data for KinBody - may need to make more general
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{

  dynamics::ShapePtr newShape;

  //Either viz(Render field) or col(Data field)
  if(hasElement(vizOrColEle,fieldName))
  {
    std::string filename = getValueString(vizOrColEle, fieldName);
    const std::string meshUri = common::Uri::getRelativeUri(_baseUri, filename);
    const aiScene* model = dynamics::MeshShape::loadMesh(meshUri, _retriever);
    const Eigen::Vector3d scale(1.0,1.0,1.0); //Default scale as kinbody does not have info
    if (model)
    {
      newShape = std::make_shared<dynamics::MeshShape>(
          scale, model, meshUri, _retriever);
    }
    else
    {
      dterr << "Fail to load model[" << filename << "]." << std::endl;
    }
  }
  else
  {
    //No field
    dterr << "[KinBodyParser::readShape] "<<fieldName<<" not present in Geom ";
    assert(0);
    return nullptr;
  }

  return newShape;
}

void readAddons(
    const dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* _KinBodyElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  ElementEnumerator xmlBodies(_KinBodyElement, "Body");
  while (xmlBodies.next())
  {
    auto bodyElement = xmlBodies.get();
    auto bodyNodeName = getAttributeString(bodyElement, "name");
    auto bodyNode = skeleton->getBodyNode(bodyNodeName);

    //Get Geom element
    tinyxml2::XMLElement* geomElement = nullptr;
    geomElement = _KinBodyElement->FirstChildElement("Geom");
    if (geomElement == nullptr)
    {
      dterr << "KinBody file[" << _baseUri.toString()
          << "] does not contain <Geom> element "
          <<"under <Body>, under <KinBody> element.\n";
      assert(0);
    }

    //Assume single viz and col shape
    readVisualizationShapeNode(bodyNode, geomElement, _baseUri, retriever);

    readCollisionShapeNode(bodyNode, geomElement, _baseUri, retriever);
  }

}


common::ResourceRetrieverPtr getRetriever(
  const common::ResourceRetrieverPtr& _retriever)
{
  if(_retriever)
    return _retriever;
  else
    return std::make_shared<common::LocalResourceRetriever>();
}

} //anonymous namespace
 
} //namespace utils

} //namespace dart