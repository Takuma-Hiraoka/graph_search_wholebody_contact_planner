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
    if (this->currentContactState->transition.size() == 0) this->currentContactState->transition.push_back(this->currentContactState->frame);
    current_node->state() = *(this->currentContactState);
    this->graph().push_back(current_node);

    return this->search();
  }

  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyContactPlanner::generateCheckParam() {
    std::shared_ptr<ContactTransitionCheckParam> checkParam = std::make_shared<ContactTransitionCheckParam>();
    cloneCheckParam(checkParam);
    return checkParam;
  }

  void WholeBodyContactPlanner::cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) {
    std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
    checkParam->bodies = std::vector<cnoid::BodyPtr>(this->bodies.size());
    for(int b=0; b<this->bodies.size(); b++){
      checkParam->bodies[b] = this->bodies[b]->clone();
      modelMap[this->bodies[b]] = checkParam->bodies[b];
      if (this->pikParam.viewer) this->pikParam.viewer->objects(checkParam->bodies[b]);
    }
    checkParam->variables = std::vector<cnoid::LinkPtr>(this->variables.size());
    for(int v=0;v<this->variables.size();v++){
      checkParam->variables[v] = modelMap[this->variables[v]->body()]->link(this->variables[v]->index());
    }
    checkParam->constraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->constraints.size());
    for(int j=0;j<this->constraints.size();j++){
      checkParam->constraints[j] = this->constraints[j]->clone(modelMap);
    }
    checkParam->rejections = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->rejections.size());
    for(int j=0;j<this->rejections.size();j++){
      checkParam->rejections[j] = this->rejections[j]->clone(modelMap);
    }
    checkParam->nominals = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->nominals.size());
    for(int j=0;j<this->nominals.size();j++){
      checkParam->nominals[j] = this->nominals[j]->clone(modelMap);
    }
    checkParam->bodyContactConstraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->bodyContactConstraints.size());
    for(int j=0;j<this->bodyContactConstraints.size();j++){
      checkParam->bodyContactConstraints[j] = this->bodyContactConstraints[j]->clone(modelMap);
    }
    checkParam->reachableConstraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->reachableConstraints.size());
    for(int j=0;j<this->reachableConstraints.size();j++){
      checkParam->reachableConstraints[j] = this->reachableConstraints[j]->clone(modelMap);
    }
    checkParam->contactDynamicCandidates = this->contactDynamicCandidates;
    checkParam->contactStaticCandidates = this->contactStaticCandidates;
    checkParam->pikParam = this->pikParam;
    checkParam->gikParam = this->gikParam;
    checkParam->robotLinkPriority = this->robotLinkPriority;
    checkParam->debugLevel = this->debugLevel();
    checkParam->addCandidateDistance = this->addCandidateDistance;
  }

  void WholeBodyContactPlanner::preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::weak_ptr<graph_search::Node> parent = extend_node->parent();
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    if (parent.expired()) contactCheckParam->preState = std::static_pointer_cast<ContactNode>(extend_node)->state(); // 初期状態なので絶対に遷移可能にしておく.
    else contactCheckParam->preState = std::static_pointer_cast<ContactNode>(parent.lock())->state();
    contactCheckParam->postState = std::static_pointer_cast<ContactNode>(extend_node)->state();
    contactCheckParam->level = std::static_pointer_cast<ContactNode>(extend_node)->level();
  }

  void WholeBodyContactPlanner::postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::static_pointer_cast<ContactNode>(extend_node)->state() = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
  }

  bool WholeBodyContactPlanner::checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    return this->checkTransitionImpl(contactCheckParam,
                                     contactCheckParam->postState);
  }


  void WholeBodyContactPlanner::addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) {
    // 再訪しない
    for (int i=0;i<this->graph().size();i++) {
      for(int j=0;j<nodes.size();j++) {
        if (std::static_pointer_cast<ContactNode>(this->graph()[i])->state() == std::static_pointer_cast<ContactNode>(nodes[j])->state()) {
          nodes.erase(nodes.begin()+j);
          break;
        }
      }
    }
    graph_search::Planner::addNodes2Graph(nodes);
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
      getchar();
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
