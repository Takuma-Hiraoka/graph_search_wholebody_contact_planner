#pragma once

#include <graph_search_wholebody_contact_planner/contact_graph.h>
#include <ik_constraint2_body_contact/BodyContactConstraint.h>
#include <c-wcp/c-wcp.h>

namespace graph_search_wholebody_contact_planner{
  class WholeBodyLocomotionContactPlanner : public WholeBodyContactPlanner { // リンクごとに接触を扱う一方で、接触点不動のためstateには接触ローカル座標を残す必要がある. このためstateはIKの誤差によって毎回違うものになり、graphではなくtree searchを行う.
  public:
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > bodyContactConstraints; // A側にlinkやcontact pointを設定しておくこと
    std::vector<std::vector<double> > guidePath; // ガイドパスのリンクとvariableは順番・個数が同じ前提

  public:
    class LocomotionContactTransitionCheckParam : public graph_search_wholebody_contact_planner::WholeBodyContactPlanner::ContactTransitionCheckParam {
    public:
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > bodyContactConstraints;
      std::vector<std::vector<double> > guidePath;
    };
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) override;
    bool solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                       Contact& moveContact,
                       ContactState& postState,
                       const IKState& ikState
                       ) override;
  };
  void convertCWCPParam(const cwcp::CWCPParam& param, WholeBodyLocomotionContactPlanner& planner);
}
