#pragma once

#include <graph_search_wholebody_contact_planner/contact_graph.h>

namespace graph_search_wholebody_contact_planner{
  class WholeBodyManipulationContactPlanner : public WholeBodyContactPlanner { // リンクごとに接触を扱う一方で、接触点不動のためstateには接触ローカル座標を残す必要がある. このためstateはIKの誤差によって毎回違うものになり、graphではなくtree searchを行う.
  public:
    std::pair<Contact, bool> targetContact; // Contactとattachかどうか

  public:
    class ManipulationContactTransitionCheckParam : public graph_search_wholebody_contact_planner::WholeBodyContactPlanner::ContactTransitionCheckParam {
    public:
      std::pair<Contact, bool> targetContact;
    };
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    bool solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                       Contact& moveContact,
                       ContactState& postState,
                       const IKState& ikState
                       ) override;
  };
}
