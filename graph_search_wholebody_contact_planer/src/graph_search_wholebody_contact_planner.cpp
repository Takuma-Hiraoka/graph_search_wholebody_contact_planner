#include <graph_search_wholebody_contact_planner/contact_graph.h>
#include <graph_search_wholebody_contact_planner/contact_node.h>
#include <graph_search_wholebody_contact_planner/util.h>

namespace graph_search_wholebody_contact_planner{
  inline std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  bool WholeBodyContactPlanner::solve() {
    std::shared_ptr<ContactNode> current_node = std::make_shared<ContactNode>();
    current_node->state() = *(param.currentContactState);
    this->graph().push_back(current_node);

    return this->search();
  }

  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyContactPlanner::generateCheckParam() {
    std::shared_ptr<ContactTransitionCheckParam> checkParam = std::make_shared<ContactTransitionCheckParam>();
    std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
    checkParam->bodies = std::vector<cnoid::BodyPtr>(param.bodies.size());
    for(int b=0; b<param.bodies.size(); b++){
      checkParam->bodies[b] = param.bodies[b]->clone();
      modelMap[param.bodies[b]] = checkParam->bodies[b];
    }
    checkParam->variables = std::vector<cnoid::LinkPtr>(param.variables.size());
    for(int v=0;v<param.variables.size();v++){
      checkParam->variables[v] = modelMap[param.variables[v]->body()]->link(param.variables[v]->index());
    }
    checkParam->constraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.constraints.size());
    for(int j=0;j<param.constraints.size();j++){
      checkParam->constraints[j] = param.constraints[j]->clone(modelMap);
    }
    checkParam->rejections = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.rejections.size());
    for(int j=0;j<param.rejections.size();j++){
      checkParam->rejections[j] = param.rejections[j]->clone(modelMap);
    }
    checkParam->nominals = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.nominals.size());
    for(int j=0;j<param.nominals.size();j++){
      checkParam->nominals[j] = param.nominals[j]->clone(modelMap);
    }
    checkParam->pikParam = param.pikParam;
    checkParam->gikParam = param.gikParam;

    return checkParam;
  }

  void WholeBodyContactPlanner::preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::weak_ptr<graph_search::Node> parent = extend_node->parent();
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    if (parent.expired()) contactCheckParam->preState = std::static_pointer_cast<ContactNode>(extend_node)->state(); // 初期状態なので絶対に遷移可能にしておく.
    else contactCheckParam->preState = std::static_pointer_cast<ContactNode>(parent.lock())->state();
    contactCheckParam->postState = std::static_pointer_cast<ContactNode>(extend_node)->state();
  }

  void WholeBodyContactPlanner::postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::static_pointer_cast<ContactNode>(extend_node)->state() = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
  }

  bool WholeBodyContactPlanner::checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    return this->checkTransitionImpl(contactCheckParam,
                                     contactCheckParam->postState);
  }

  bool WholeBodyContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    ContactState state = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
    for (int i=0;i<this->param.goalContactState->contacts.size();i++) {
      bool satisfied = false;
      for (int j=0;j<state.contacts.size();j++) {
        if (this->param.goalContactState->contacts[i] == state.contacts[j]) satisfied = true;
      }
      if (!satisfied) return false;
    }
    return true;
  }

  void WholeBodyContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) {
    // goalContactStateのcontactsのうち満たしていないもの
    // TODO 追加する
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    int unsatisfied_num = 0;
    for (int i=0;i<this->param.goalContactState->contacts.size();i++) {
      bool satisfied = false;
      for (int j=0;j<state.contacts.size();j++) {
        if (this->param.goalContactState->contacts[i] == state.contacts[j]) satisfied = true;
      }
      if (!satisfied) unsatisfied_num++;
    }
    node->heuristic() = unsatisfied_num;
  }

  std::vector<std::shared_ptr<graph_search::Node> > WholeBodyContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    ContactState extend_state = std::static_pointer_cast<ContactNode>(extend_node)->state();
    if (this->debugLevel() >= 2) {
      std::cerr << "extend_state" << std::endl;
      std::cerr << extend_state << std::endl;
    }
    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->parent() = extend_node;
        newNode->state() = extend_state;
        newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
        adjacentNodes.push_back(newNode);
      }
    }

    // static contact
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      // staticCandidateと接触しているものを更にstaticCandidateと接触させることはしない
      bool skip = false;
      for (int j=0; j<extend_state.contacts.size(); j++) {
        if (((param.contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c1.bodyName) && (param.contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c1.linkName) && (extend_state.contacts[j].c2.isStatic)) ||
            ((param.contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c2.bodyName) && (param.contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c2.linkName) && (extend_state.contacts[j].c1.isStatic))) {
          skip = true;
          break;
        }
      }
      if (skip) continue;

      // ルートリンク位置からaddCandidateDistanceを超える距離のstaticCandidateと接触させることはしない
      // 高速化のため. gikを使うまでもなく解けない
      cnoid::Vector3 rootPos;
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if ((contactCheckParam->bodies[b]->name() == param.contactDynamicCandidates[i]->bodyName) && contactCheckParam->bodies[b]->joint(param.contactDynamicCandidates[i]->linkName)) rootPos = contactCheckParam->bodies[b]->rootLink()->p();
      }
      for (int j=0; j<param.contactStaticCandidates.size(); j++) {
        if ((rootPos - param.contactStaticCandidates[j]->localPose.translation()).norm() > param.addCandidateDistance) continue;

        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->parent() = extend_node;
        newNode->state() = extend_state;
        Contact c = Contact(*(param.contactDynamicCandidates[i]), *(param.contactStaticCandidates[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    // dynamic contact
    std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates;
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      // 既に接触している接触候補は接触できない
      bool in_contact = false;
      for (int j=0; j<extend_state.contacts.size() && !in_contact; j++) {
        if (((param.contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c1.bodyName) && (param.contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c1.linkName) && (param.contactDynamicCandidates[i]->localPose.translation() == extend_state.contacts[j].c1.localPose.translation())) ||
            ((param.contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c2.bodyName) && (param.contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c2.linkName) && (param.contactDynamicCandidates[i]->localPose.translation() == extend_state.contacts[j].c2.localPose.translation()))) in_contact = true;
      }
      if (!in_contact) contactDynamicCandidates.push_back(param.contactDynamicCandidates[i]);
    }

    for (int i=0; i<contactDynamicCandidates.size(); i++) {
      // それぞれのルートリンク位置の距離がaddCandidateDistanceを超えるcontactDynamicCandidate同士を接触させることはしない
      // 高速化のため. gikを使うまでもなく解けない
      cnoid::Vector3 rootPos1;
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if ((contactCheckParam->bodies[b]->name() == param.contactDynamicCandidates[i]->bodyName) && contactCheckParam->bodies[b]->joint(param.contactDynamicCandidates[i]->linkName)) rootPos1 = contactCheckParam->bodies[b]->rootLink()->p();
      }
      for (int j=i+1; j<contactDynamicCandidates.size(); j++) {
        // 同じリンク内の候補同士は接触できない
        if ((contactDynamicCandidates[i]->bodyName == contactDynamicCandidates[j]->bodyName) && (contactDynamicCandidates[i]->linkName == contactDynamicCandidates[j]->linkName)) continue;
        cnoid::Vector3 rootPos2;
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if ((contactCheckParam->bodies[b]->name() == param.contactDynamicCandidates[j]->bodyName) && contactCheckParam->bodies[b]->joint(param.contactDynamicCandidates[j]->linkName)) rootPos2 = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->bodies[b]->rootLink()->p();
        }
        if ((rootPos1 - rootPos2).norm() > param.addCandidateDistance) continue;

        // localPoseが違ったとしても既に接触しているリンク同士を更に接触させることはしない
        bool found = false;
        for (int k=0; k<extend_state.contacts.size() && !found; k++) {
          if (extend_state.contacts[k] == Contact(*(contactDynamicCandidates[i]), *(contactDynamicCandidates[j]))) found = true;
        }
        if (found) continue;

        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->parent() = extend_node;
        newNode->state() = extend_state;
        Contact c = Contact(*(contactDynamicCandidates[i]), *(contactDynamicCandidates[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    // 再訪しない
    for (int i=0;i<this->graph().size();i++) {
      for(int j=0;j<adjacentNodes.size();j++) {
        if (std::static_pointer_cast<ContactNode>(this->graph()[i])->state() == std::static_pointer_cast<ContactNode>(adjacentNodes[j])->state()) {
          adjacentNodes.erase(adjacentNodes.begin()+j);
          break;
        }
      }
    }

    return adjacentNodes;
  }

  bool WholeBodyContactPlanner::checkTransitionImpl(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                                                    ContactState& postState
                                                    ) {
    if (this->debugLevel() >= 2) {
      std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition" << std::endl;
      std::cerr << "preState" << std::endl;
      std::cerr << checkParam->preState << std::endl;
      std::cerr << "postState" << std::endl;
      std::cerr << postState << std::endl;
    }

    global_inverse_kinematics_solver::frame2Link(checkParam->preState.frame, checkParam->variables);
    postState.transition.clear();

    if (postState.contacts.size() > checkParam->preState.contacts.size()) {
      // attach
      if (!solveContactIK(checkParam, postState.contacts.back(), postState, IKState::ATTACH_PRE)) return false;
      if (!solveContactIK(checkParam, postState.contacts.back(), postState, IKState::ATTACH_FIXED)) return false;
    } else if (postState.contacts.size() < checkParam->preState.contacts.size()) {
      // detach
      bool find_detach_contact = false;
      Contact moveContact;
      for(int i=0;i<checkParam->preState.contacts.size() && !find_detach_contact;i++) {
        if(std::find(postState.contacts.begin(), postState.contacts.end(), checkParam->preState.contacts[i]) == postState.contacts.end()) {
          moveContact = checkParam->preState.contacts[i];
          find_detach_contact = true;
        }
      }
      if (!find_detach_contact) {
        std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition failed!! cannot find detach contact" << std::endl;
        return false;
      }
      if (!solveContactIK(checkParam, moveContact, postState, IKState::DETACH_FIXED)) return false;
    } else {
      if (checkParam->preState == postState) {
        // 同じ. はじめの接触.
        postState.transition.push_back(checkParam->preState.frame);
        return true;
      }
      std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition failed!! postState.contacts.size() is same as preState.contacts.size()" << std::endl;
      return false;
    }
    global_inverse_kinematics_solver::link2Frame(checkParam->variables, postState.frame);
    return true;
  }

  void WholeBodyContactPlanner::goalPath(std::vector<ContactState>& path) {
    if (!this->goal()) {
      std::cerr << "[WholeBodyContactPlanner] goal not found!!" << std::endl;
    } else {
      path.clear();
      path.push_back(std::static_pointer_cast<graph_search_wholebody_contact_planner::ContactNode>(this->goal())->state());
      std::weak_ptr<graph_search::Node> node = this->goal()->parent();
      while (!node.expired()) {
        path.push_back(std::static_pointer_cast<graph_search_wholebody_contact_planner::ContactNode>(node.lock())->state());
        node = node.lock()->parent();
      }
    }
    std::reverse(path.begin(), path.end());
  }

}
