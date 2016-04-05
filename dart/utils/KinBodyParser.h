#ifndef DART_UTILS_KINBODY_PARSER_H
#define DART_UTILS_KINBODY_PARSER_H

#include <cstddef>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/common/Uri.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/simulation/World.h"

//CURRENTLY ASSUMES ONLY ONE BODY NODE

namespace dart {

namespace utils{

namespace KinBodyParser {

  dynamics::SkeletonPtr readKinBodyXMLFile(
    const common::Uri& _fileUri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);


} //namespace KinBodyParser

} //namespace utils

} //namespace dart

#endif // #ifndef DART_UTILS_KINBODY_PARSER_H