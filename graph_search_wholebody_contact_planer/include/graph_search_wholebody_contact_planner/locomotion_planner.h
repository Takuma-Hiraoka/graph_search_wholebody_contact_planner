#pragma once

#include <graph_search_wholebody_contact_planner/contact_graph.h>

namespace graph_search_wholebody_contact_planner{
  class WholeBodyLocomotionContactPlanner : public WholeBodyContactPlanner {
  public:
    class LocomotionContactTransitionCheckParam : public graph_search_wholebody_contact_planner::WholeBodyContactPlanner::ContactTransitionCheckParam {
    public:
      std::vector<std::vector<double> > guidePath;
    };
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) override;
    bool solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                       Contact& moveContact,
                       ContactState& postState,
                       const IKState& ikState
                       ) override;
    class GSWLCPParam : WholeBodyContactPlanner::GSWCPParam {
    public:
      std::vector<std::vector<double> > guidePath; // ガイドパスのリンクとvariableは順番・個数が同じ前提
    };
    GSWLCPParam param;
  };
}
