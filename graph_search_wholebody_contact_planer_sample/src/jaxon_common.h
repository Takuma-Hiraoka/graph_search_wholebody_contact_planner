#pragma once

#include <c-wcp/c-wcp.h>
#include <choreonoid_contact_candidate_generator/choreonoid_contact_candidate_generator.h>

namespace graph_search_wholebody_contact_planner_sample{
  void generateJAXON(cnoid::BodyPtr& robot, std::shared_ptr<cwcp::CWCPParam>& param);
  std::shared_ptr<ik_constraint2::IKConstraint> generateBodyContactConstraint(std::vector<cnoid::BodyPtr>& bodies, cnoid::LinkPtr link, double resolution);
};
