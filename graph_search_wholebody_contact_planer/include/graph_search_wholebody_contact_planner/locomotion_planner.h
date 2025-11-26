#pragma once

#include <graph_search_wholebody_contact_planner/contact_graph.h>
#include <c-wcp/c-wcp.h>

namespace graph_search_wholebody_contact_planner{
  class WholeBodyLocomotionContactPlanner : public WholeBodyContactPlanner { // リンクごとに接触を扱う一方で、接触点不動のためstateには接触ローカル座標を残す必要がある. このためstateはIKの誤差によって毎回違うものになり、graphではなくtree searchを行う.
  public:
    double addNearGuideCandidateDistance = 0.2;
    std::vector<std::pair<std::vector<double>, std::vector<Contact> > > guidePath; // ガイドパスのframeはリンクとvariableは順番・個数が同じ前提
    std::string targetBodyName;

  public:
    class LocomotionContactTransitionCheckParam : public graph_search_wholebody_contact_planner::WholeBodyContactPlanner::ContactTransitionCheckParam {
    public:
      double addNearGuideCandidateDistance = 0.2;
      std::vector<std::pair<std::vector<double>, std::vector<Contact> > > guidePath;
      std::string targetBodyName;
    };
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void candidatesFromGuide(const std::vector<cnoid::BodyPtr> bodies, const std::vector<std::shared_ptr<ContactCandidate> >& contactCandidatesRaw, const std::vector<std::pair<std::vector<double>, std::vector<Contact> > >& guidePath, const std::string& bodyName, const std::string& linkName, std::vector<std::shared_ptr<ContactCandidate> >& candidates, const int& level=0);
    bool solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                       Contact& moveContact,
                       ContactState& postState,
                       const IKState& ikState
                       ) override;
  };
  void convertParamFromCWCP(const cwcp::CWCPParam& param, const std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > >& cwcpPath, WholeBodyLocomotionContactPlanner& planner);
  void convertDetachParamToCWCP(const WholeBodyContactPlanner& planner, const std::vector<Contact>& currentContacts, const cnoid::BodyPtr robot, const cnoid::BodyPtr object, const std::vector<std::shared_ptr<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint> > reachabilityConstraints, cwcp::CWCPParam& param, std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& removedConstraints, std::vector<Contact>& stableContacts);
}
