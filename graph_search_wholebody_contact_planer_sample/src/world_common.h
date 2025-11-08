#pragma once
#include <cnoid/Body>
#include <c-wcp/c-wcp.h>

namespace graph_search_wholebody_contact_planner_sample{
  void generateWallWorld(cnoid::BodyPtr& obstacle, // for visual
                         const std::shared_ptr<cwcp::CWCPParam>& param);

};
