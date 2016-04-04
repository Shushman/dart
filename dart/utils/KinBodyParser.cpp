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
#include "dart/utils/KinBodyParser.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"

namespace dart {
namespace utils {

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
    openXMLFile(_kinBodyFile, _fileUri, retriever);
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile [" << _fileUri.toString() << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  tinyxml2::XMLElement* kinBodyElement = nullptr;
  kinBodyElement = _dartFile.FirstChildElement("KinBody");
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

dynamics::SkeletonPtr KinBodyParser::readKinBody(
  tinyxml2::XMLElement* _KinBodyElement,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever);
{

  assert(_KinBodyElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttribute(_KinBodyElement, "name");
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
    bodyElement, skeletonFrame,_baseUri, retriever);

  //How to add newBodyNode to new Skeleton?
  //Also need to add some static joint I guess
  //JS HELP!


  return newSkeleton

}

KinBodyParser::SkelBodyNode KinBodyParser::readBodyNode(
  tinyxml2::XMLElement* _bodyNodeElement,
  const Eigen::Isometry3d& _skeletonFrame,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _retriever)
{
  assert(_bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = getAttribute(_bodyNodeElement, "name");

  //Get Geom element
  tinyxml2::XMLElement* geomElement = nullptr;
  geomElement = _bodyNodeElement->FirstChildElement("Geom");
  if (geomElement == nullptr)
  {
    dterr << "KinBody file[" << _baseUri.toString()
          << "] does not contain <Geom> element "
          <<"under <Body>, under <KinBody> element.\n";
    return nullptr;
  }

  //Get collision and visualizing data with readShape
  dynamics::ShapePtr vizShape = readShape(geomElement,"Render",_baseUri,_retriever);
  dynamics::ShapePtr colShape = readShape(geomElement,"Data",_baseUri,_retriever);

  //Add colShape and vizShape to newBodyNode
  newBodyNode->mVizShapes.push_back(vizShape);
  newBodyNode->mColShapes.push_back(colShape);

  //Get SkelBodyNode from PropPtr
  SkelBodyNode skelBodyNode;
  skelBodyNode.properties = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}


//Similar to SkelParser::readShape
dynamics::ShapePtr KinBodyParser::readShape(
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
    dterr << "[KinBodyParser::readShape] "<<fieldName<<" not present in Geom for [" 
      << bodyName << "]\n";
    assert(0);
    return nullptr;
  }

  return newShape;

}