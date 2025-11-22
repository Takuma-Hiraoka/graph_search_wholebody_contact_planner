#pragma once
#include <cnoid/Body>
#include <c-wcp/c-wcp.h>

namespace graph_search_wholebody_contact_planner_sample{
  void generateWallWorld(cnoid::BodyPtr& obstacle,
                         const std::shared_ptr<cwcp::CWCPParam>& param);
  void generateTableWorld(cnoid::BodyPtr& obstacle,
                          const std::shared_ptr<cwcp::CWCPParam>& param);
  void generateHallWorld(cnoid::BodyPtr& obstacle,
                         const std::shared_ptr<cwcp::CWCPParam>& param);
  void generateBOX(cnoid::BodyPtr& box,
                   const std::shared_ptr<cwcp::CWCPParam>& param);
  void generateCUBE(cnoid::BodyPtr& cube,
                    const std::shared_ptr<cwcp::CWCPParam>& param);
}
