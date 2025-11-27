#include <graph_search_wholebody_contact_planner/locomotion_planner.h>
#include <ik_constraint2_scfr/ik_constraint2_scfr.h>
#include <limits>
#include <random>

namespace graph_search_wholebody_contact_planner{
  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyLocomotionContactPlanner::generateCheckParam() {
    std::shared_ptr<LocomotionContactTransitionCheckParam> checkParam = std::make_shared<LocomotionContactTransitionCheckParam>();
    cloneCheckParam(checkParam);
    return checkParam;
  }

  void WholeBodyLocomotionContactPlanner::cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) {
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    WholeBodyContactPlanner::cloneCheckParam(contactCheckParam);
    contactCheckParam->guidePath = this->guidePath;
    contactCheckParam->addNearGuideCandidateDistance = this->addNearGuideCandidateDistance;
    contactCheckParam->targetBodyName = this->targetBodyName;
  }

  inline std::pair<double, int> calcContactDiff(const std::vector<cnoid::BodyPtr>& bodies, const Contact& contact, const std::vector<std::pair<std::vector<double>, std::vector<Contact> > >& guidePath) {
    cnoid::Isometry3 c1Pose = contact.c1.localPose;
    cnoid::Isometry3 c2Pose = contact.c2.localPose;
    for (int b=0; b<bodies.size(); b++) {
      if ((bodies[b]->name() == contact.c1.bodyName) && bodies[b]->link(contact.c1.linkName)) c1Pose = bodies[b]->link(contact.c1.linkName)->T() * contact.c1.localPose;
      if ((bodies[b]->name() == contact.c2.bodyName) && bodies[b]->link(contact.c2.linkName)) c2Pose = bodies[b]->link(contact.c2.linkName)->T() * contact.c2.localPose;
    }
    int nearestIdx = -1; // guidePathにない接触は-1で示す. objectを持った状態でのguidePath等
    double diffMin = std::numeric_limits<double>::max();
    double contactWeight = 1e1;
    for (int i=0; i<guidePath.size(); i++) {
      double diff = 0;
      unsigned int idx=0;
      bool found = false;
      for (int c=0; c<guidePath[i].second.size() && !found; c++) {
        cnoid::Isometry3 c2gPose = guidePath[i].second[c].c2.localPose;
        for (int b=0; b<bodies.size(); b++) {
          if ((bodies[b]->name() == guidePath[i].second[c].c2.bodyName) && bodies[b]->link(guidePath[i].second[c].c2.linkName)) c2gPose = bodies[b]->link(guidePath[i].second[c].c2.linkName)->T() * guidePath[i].second[c].c2.localPose;
        }
        if ((contact.c1.bodyName == guidePath[i].second[c].c1.bodyName) && (contact.c1.linkName == guidePath[i].second[c].c1.linkName) && contact.c2.isStatic) {
          found = true;
          diff += (c2Pose.translation() - c2gPose.translation()).norm() * contactWeight;
        } else if ((contact.c2.bodyName == guidePath[i].second[c].c1.bodyName) && (contact.c2.linkName == guidePath[i].second[c].c1.linkName) && contact.c1.isStatic) {
          found = true;
          diff += (c1Pose.translation() - c2gPose.translation()).norm() * contactWeight;
        }
      }
      if (found && (diff < diffMin)) {
        diffMin = diff;
        nearestIdx = i;
      }
    }
    return std::make_pair(diffMin, nearestIdx);
  }

  bool WholeBodyLocomotionContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    ContactState state = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);

    double diff = 0.0;
    double rootWeight = 1e2;
    unsigned int idx=0;
    for (int l=0; l<contactCheckParam->variables.size(); l++) {
      if (variables[l]->isRevoluteJoint() || variables[l]->isPrismaticJoint()) {
        //diff += std::abs(state.frame[idx] - guidePath[i].first[idx]);
        idx+=1;
      } else if (contactCheckParam->variables[l]->isFreeJoint()) {
        if (variables[l]->body()->name() == contactCheckParam->targetBodyName) diff += (cnoid::Vector3(state.frame[idx+0], state.frame[idx+1], state.frame[idx+2]) - cnoid::Vector3(contactCheckParam->guidePath.back().first[idx+0], contactCheckParam->guidePath.back().first[idx+1], contactCheckParam->guidePath.back().first[idx+2])).norm() * rootWeight;
        // diff += std::abs(cnoid::AngleAxis(cnoid::Quaternion(state.frame[idx+6],state.frame[idx+3],state.frame[idx+4],state.frame[idx+5]).toRotationMatrix().transpose() * cnoid::Quaternion(contactCheckParam->guidePath.back().first[idx+6],contactCheckParam->guidePath.back().first[idx+3],contactCheckParam->guidePath.back().first[idx+4],contactCheckParam->guidePath.back().first[idx+5]).toRotationMatrix()).angle()) * 1e1;
        idx+=7;
      }
    }

    // std::cerr << "diff " << diff << std::endl;
    return diff < 1e1;
  }

  void WholeBodyLocomotionContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) {
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    if (state.transition.size() < 1) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.transition.size() < 1" << std::endl;
    if (state.transition[0].size() != state.frame.size()) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.transition[0].size() and state.frame.size() are mismatched!" << std::endl;
    if (state.frame.size() != contactCheckParam->guidePath[0].first.size()) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.frame.size() and contactCheckParam.guidePath[0].size() are mismatched!" << std::endl;
    double diffSum = 0;
    double nearestIdxSum = 0;
    int count = 0;
    for (int i=0; i<state.contacts.size(); i++) {
      std::pair<double, int> contactDiff = calcContactDiff(contactCheckParam->bodies, state.contacts[i], contactCheckParam->guidePath);
      if (contactDiff.second == -1) continue;
      diffSum += contactDiff.first;
      nearestIdxSum += contactDiff.second;
      count++;
    }
    if (count == 0) node->heuristic() = contactCheckParam->guidePath.size() * 1e3; // ガイドパスに出てくる接触がない. objectを持つ接触だけ等. robotのSCFRがないので実行不可.
    else {
      double nearestIdx = (nearestIdxSum / count);

      std::cerr << "path size " << contactCheckParam->guidePath.size() << " nearestIdx : " << nearestIdx << std::endl;
      std::static_pointer_cast<ContactNode>(node)->level() = (unsigned int)nearestIdx;
      node->heuristic() = (contactCheckParam->guidePath.size() - 1 - nearestIdx) * 1e3 + diffSum;
    }
  }

  std::vector<std::shared_ptr<graph_search::Node> > WholeBodyLocomotionContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    ContactState extend_state = contactCheckParam->postState;
    if (contactCheckParam->debugLevel >= 2) {
      std::cerr << "extend_state" << std::endl;
      std::cerr << extend_state << std::endl;
    }
    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->state() = extend_state;
        newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
        adjacentNodes.push_back(newNode);
      }
    }

    // static contact
    for (int i=0; i<contactCheckParam->contactDynamicCandidates.size(); i++) {
      // staticCandidateと接触しているものを更にstaticCandidateと接触させることはしない
      bool skip = false;
      for (int j=0; j<extend_state.contacts.size(); j++) {
        if (((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c1.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c1.linkName) && (extend_state.contacts[j].c2.isStatic)) ||
            ((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c2.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c2.linkName) && (extend_state.contacts[j].c1.isStatic))) {
          skip = true;
          break;
        }
      }
      if (skip) continue;

      // ルートリンク位置からaddCandidateDistanceを超える距離のstaticCandidateと接触させることはしない
      // 高速化のため. gikを使うまでもなく解けない
      cnoid::Vector3 rootPos;
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if ((contactCheckParam->bodies[b]->name() == contactCheckParam->contactDynamicCandidates[i]->bodyName) && contactCheckParam->bodies[b]->joint(contactCheckParam->contactDynamicCandidates[i]->linkName)) rootPos = contactCheckParam->bodies[b]->rootLink()->p();
      }

      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidatesLimited;
      for (int j=0; j<contactCheckParam->contactStaticCandidates.size(); j++) {
        if ((rootPos - contactCheckParam->contactStaticCandidates[j]->localPose.translation()).norm() > contactCheckParam->addCandidateDistance) continue;
        contactStaticCandidatesLimited.push_back(contactCheckParam->contactStaticCandidates[j]);
      }
      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidatesNear;
      candidatesFromGuide(contactCheckParam->bodies, contactStaticCandidatesLimited, contactCheckParam->guidePath, contactCheckParam->contactDynamicCandidates[i]->bodyName, contactCheckParam->contactDynamicCandidates[i]->linkName, contactStaticCandidatesNear, contactCheckParam->level);

      for (int j=0; j<contactStaticCandidatesNear.size(); j++) {
        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->state() = extend_state;
        Contact c = Contact(*(contactCheckParam->contactDynamicCandidates[i]), *(contactStaticCandidatesNear[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    // dynamic contact
    // locomotionにおいては、dynamicとdynamicは接触しない

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(adjacentNodes.begin(), adjacentNodes.end(), g);
    return adjacentNodes;

  }

  void WholeBodyLocomotionContactPlanner::candidatesFromGuide(const std::vector<cnoid::BodyPtr> bodies, const std::vector<std::shared_ptr<ContactCandidate> >& contactCandidatesRaw, const std::vector<std::pair<std::vector<double>, std::vector<Contact> > >& guidePath, const std::string& bodyName, const std::string& linkName, std::vector<std::shared_ptr<ContactCandidate> >& candidates, const int& level) {
    // guidePathに従って探索する範囲を制限する
    // guidePathの全ノードについて、contactableになっているリンクのcontactDynamicCandidatesと、最近接している環境点からaddNearGuideCandidateDistance以内の距離のstaticCandidateを接触させる
    for (int j=0; j<contactCandidatesRaw.size(); j++) {
      for (int g=level; g<guidePath.size(); g++) {
        for (int c=0; c<guidePath[g].second.size(); c++) {
          if ((bodyName == guidePath[g].second[c].c1.bodyName) && (linkName == guidePath[g].second[c].c1.linkName)) // guidePathのこのnodeでcontactDynamicCandidates[i]はcontactable
            {
              cnoid::Isometry3 guidePose = guidePath[g].second[c].c2.localPose;
              cnoid::Isometry3 candidatePose = contactCandidatesRaw[j]->localPose;
              for (int b=0; b < bodies.size(); b++) {
                if ((bodies[b]->name() == guidePath[g].second[c].c2.bodyName) && bodies[b]->link(guidePath[g].second[c].c2.linkName)) {
                  guidePose = bodies[b]->link(guidePath[g].second[c].c2.linkName)->T() * guidePath[g].second[c].c2.localPose;
                }
                if ((bodies[b]->name() == contactCandidatesRaw[j]->bodyName) && bodies[b]->link(contactCandidatesRaw[j]->linkName)) {
                  candidatePose = bodies[b]->link(contactCandidatesRaw[j]->linkName)->T() * contactCandidatesRaw[j]->localPose;
                }
              }
              if (((guidePose.translation() - candidatePose.translation()).norm() <= this->addNearGuideCandidateDistance) && // 最近接している環境点からaddNearGuideCandidateDistance以内
                  (cnoid::AngleAxisd(guidePose.linear().transpose() * candidatePose.linear()).angle() >= M_PI / 2)) // 姿勢. もともと反転していることに注意
              {
                candidates.push_back(contactCandidatesRaw[j]);
              }
        }
      }
    }
  }

  }

  bool WholeBodyLocomotionContactPlanner::solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                                                         Contact& moveContact,
                                                         ContactState& postState,
                                                         const IKState& ikState
                                                         ) {
    std::shared_ptr<const WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<const WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    std::shared_ptr<std::vector<std::vector<double> > > tmpPath = std::make_shared<std::vector<std::vector<double> > >();
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0; // 幾何干渉や重心制約.
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1; // 動かさない接触
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2; // 動かす接触
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;

    for (int i=0; i<contactCheckParam->constraints.size(); i++) {
      bool skip=false;
      if (typeid(*(contactCheckParam->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        std::shared_ptr<ik_constraint2::CollisionConstraint> constraint = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(contactCheckParam->constraints[i]);
        for (int j=0; j<contactCheckParam->preState.contacts.size() && !skip; j++) {
          if (((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->A_link()->name()) && contactCheckParam->preState.contacts[j].c2.isStatic) ||
              ((contactCheckParam->preState.contacts[j].c2.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->A_link()->name()) && contactCheckParam->preState.contacts[j].c1.isStatic)) skip = true;
        }
        if (!skip && ((ikState==IKState::ATTACH_FIXED) ||
                      (ikState==IKState::DETACH_FIXED))) {
          if (((moveContact.c1.bodyName == constraint->A_link()->body()->name()) && (moveContact.c1.linkName == constraint->A_link()->name())) ||
              ((moveContact.c2.bodyName == constraint->A_link()->body()->name()) && (moveContact.c2.linkName == constraint->A_link()->name()))) skip = true;
        }
      }
      if (typeid(*(contactCheckParam->constraints[i]))==typeid(ik_constraint2_bullet::BulletCollisionConstraint)) {
        std::shared_ptr<ik_constraint2::CollisionConstraint> constraint = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(contactCheckParam->constraints[i]);
        for (int j=0; j<contactCheckParam->preState.contacts.size() && !skip; j++) {
          if (((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->A_link()->name()) && (contactCheckParam->preState.contacts[j].c2.bodyName == constraint->B_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->B_link()->name())) ||
              ((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->B_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->B_link()->name()) && (contactCheckParam->preState.contacts[j].c2.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->A_link()->name()))) skip = true;
        }
        if (!skip && ((ikState==IKState::ATTACH_FIXED) ||
                      (ikState==IKState::DETACH_FIXED))) {
          if (((moveContact.c1.bodyName == constraint->A_link()->body()->name()) && (moveContact.c1.linkName == constraint->A_link()->name()) && (moveContact.c2.bodyName == constraint->B_link()->body()->name()) && (moveContact.c2.linkName == constraint->B_link()->name())) ||
              ((moveContact.c2.bodyName == constraint->A_link()->body()->name()) && (moveContact.c2.linkName == constraint->A_link()->name()) && (moveContact.c1.bodyName == constraint->B_link()->body()->name()) && (moveContact.c1.linkName == constraint->B_link()->name()))) skip = true;
        }
      }
      if (!skip) constraints0.push_back(contactCheckParam->constraints[i]);
    }
    // scfrConstraint
    std::vector<std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> > scfrConstraints;
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->rootLink()->isFreeJoint() && // staticな環境ではない
          contactCheckParam->bodies[b]->name().find("body_contact_explore") == std::string::npos) // 接触点探索用のbodyではない
        {
        std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
        scfrConstraint->A_robot() = contactCheckParam->bodies[b];
        scfrConstraints.push_back(scfrConstraint);
      }
    }

    {
      for (int i=0; i<contactCheckParam->preState.contacts.size(); i++) {
        if (contactCheckParam->preState.contacts[i] == moveContact) continue;
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c1.bodyName) continue;
          if (contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)) {
            constraint->A_link() = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName);
            break;
          }
        }
        constraint->A_localpos() = contactCheckParam->preState.contacts[i].c1.localPose;
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) continue;
          if (contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)) {
            constraint->B_link() = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName);
            break;
          }
        }
        constraint->B_localpos() = contactCheckParam->preState.contacts[i].c2.localPose;
        constraint->B_localpos().linear() = constraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるためにZの向きを揃える.
        constraint->eval_link() = constraint->B_link();
        constraint->eval_localR() = constraint->B_localpos().linear();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.5;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.5;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.5;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.5;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.1; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.1; C.insert(10,5) = -1.0;
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        for (int j=0;j<scfrConstraints.size();j++) {
          // Linkの位置から出す場合、位置固定でも数値誤差によって姿勢が少しずつずれていき、scfr計算の線型計画法に不具合が生じてscfrの領域が潰れる.
          // これを避けるため、staticのときは環境側の接触情報を使う. dynamicのときはbodyごとに2つ以上の接触がありscfrが残ると期待.
          if ((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c1.bodyName) && scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c1.linkName)) {
            if (contactCheckParam->preState.contacts[i].c2.isStatic) {
              scfrConstraints[j]->links().push_back(nullptr);
              cnoid::Isometry3 pose = contactCheckParam->preState.contacts[i].c2.localPose;
              for (int b=0; b < contactCheckParam->bodies.size(); b++) {
                if ((contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) && contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)) pose = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)->T() * contactCheckParam->preState.contacts[i].c2.localPose;
              }
              pose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose();
              scfrConstraints[j]->poses().push_back(pose);
            } else {
              scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c1.linkName));
              scfrConstraints[j]->poses().push_back(contactCheckParam->preState.contacts[i].c1.localPose);
            }
            scfrConstraints[j]->As().emplace_back(0,6);
            scfrConstraints[j]->bs().emplace_back(0);
            scfrConstraints[j]->Cs().push_back(C);
            scfrConstraints[j]->dls().push_back(dl);
            scfrConstraints[j]->dus().push_back(du);
          }
          if ((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c2.bodyName) && scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c2.linkName)) {
            if (contactCheckParam->preState.contacts[i].c1.isStatic) {
              scfrConstraints[j]->links().push_back(nullptr);
              cnoid::Isometry3 pose = contactCheckParam->preState.contacts[i].c1.localPose;
              for (int b=0; b < contactCheckParam->bodies.size(); b++) {
                if ((contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c1.bodyName) && contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)) pose = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)->T() * contactCheckParam->preState.contacts[i].c1.localPose;
              }
              pose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose();
              scfrConstraints[j]->poses().push_back(pose);
            } else {
              scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c2.linkName));
              scfrConstraints[j]->poses().push_back(contactCheckParam->preState.contacts[i].c2.localPose);
            }
            scfrConstraints[j]->As().emplace_back(0,6);
            scfrConstraints[j]->bs().emplace_back(0);
            scfrConstraints[j]->Cs().push_back(C);
            scfrConstraints[j]->dls().push_back(dl);
            scfrConstraints[j]->dus().push_back(du);
          }
        }
      }
    }

    std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> moveContactConstraint = std::make_shared<ik_constraint2_body_contact::BodyContactConstraint>();
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->name() != moveContact.c1.bodyName) continue;
      if (contactCheckParam->bodies[b]->joint(moveContact.c1.linkName)) {
        moveContactConstraint->A_link() = contactCheckParam->bodies[b]->joint(moveContact.c1.linkName);
        break;
      }
    }
    moveContactConstraint->A_localpos() = moveContact.c1.localPose;
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->name() != moveContact.c2.bodyName) continue;
      if (contactCheckParam->bodies[b]->joint(moveContact.c2.linkName)) {
        moveContactConstraint->B_link() = contactCheckParam->bodies[b]->joint(moveContact.c2.linkName);
        break;
      }
    }
    moveContactConstraint->B_localpos() = moveContact.c2.localPose;
    moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.

    if ((ikState==IKState::DETACH_FIXED) ||
        (ikState==IKState::ATTACH_PRE)) {
      if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
      else moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
    }
    moveContactConstraint->eval_link() = moveContactConstraint->B_link();
    moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
    moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
    constraints2.push_back(moveContactConstraint);

    unsigned int nominalIdx=0;
    unsigned int targetLevel=0;
    if (ikState == IKState::DETACH_FIXED) targetLevel = std::min(contactCheckParam->level+1, (unsigned int)contactCheckParam->guidePath.size()-1);
    if ((ikState == IKState::ATTACH_PRE) || (ikState == IKState::ATTACH_FIXED)) targetLevel = std::min(contactCheckParam->level+1, (unsigned int)contactCheckParam->guidePath.size()-1);
    for (int i=0; i<contactCheckParam->variables.size(); i++) {
      if (contactCheckParam->variables[i]->isRevoluteJoint() || contactCheckParam->variables[i]->isPrismaticJoint()) {
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = contactCheckParam->variables[i];
        constraint->targetq() = contactCheckParam->guidePath[targetLevel].first[nominalIdx];
        constraint->precision() = 1e10; // always satisfied
        nominals.push_back(constraint);
        nominalIdx += 1;
      } else if (contactCheckParam->variables[i]->isFreeJoint()) {
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = contactCheckParam->variables[i];
        cnoid::Isometry3 pose;
        pose.translation() = cnoid::Vector3(contactCheckParam->guidePath[targetLevel].first[nominalIdx+0], contactCheckParam->guidePath[targetLevel].first[nominalIdx+1], contactCheckParam->guidePath[targetLevel].first[nominalIdx+2]);
        pose.linear() = cnoid::Quaternion(contactCheckParam->guidePath[targetLevel].first[nominalIdx+6], contactCheckParam->guidePath[targetLevel].first[nominalIdx+3], contactCheckParam->guidePath[targetLevel].first[nominalIdx+4], contactCheckParam->guidePath[targetLevel].first[nominalIdx+5]).toRotationMatrix();
        constraint->B_localpos() = pose;
        if (variables[i]->body()->name() == contactCheckParam->targetBodyName) constraint->weight() << 10.0, 10.0, 10.0, 1.0, 1.0, 1.0;
        else constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraint->precision() = 1e10; // always satisfied
        nominals.push_back(constraint);
        nominalIdx += 7;
      }
    }

    for (int i=0;i<scfrConstraints.size();i++) {
      if (scfrConstraints[i]->poses().size() == 0) return false; // 接触が存在しない物体がある.
      if (!ik_constraint2_keep_collision_scfr::checkSCFRExistance(scfrConstraints[i]->poses(), scfrConstraints[i]->As(), scfrConstraints[i]->bs(), scfrConstraints[i]->Cs(), scfrConstraints[i]->dls(), scfrConstraints[i]->dus(), scfrConstraints[i]->A_robot()->mass(), scfrConstraints[i]->SCFRParam())) return false; // scfrが消えている
      constraints0.push_back(scfrConstraints[i]);
    }

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint{constraints0, constraints1, constraints2, nominals};

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(contactCheckParam->variables,
                                                                   constraint,
                                                                   contactCheckParam->rejections,
                                                                   prevTasks,
                                                                   contactCheckParam->pikParam,
                                                                   tmpPath
                                                                   );
    if(!solved) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      global_inverse_kinematics_solver::GIKParam gikParam = contactCheckParam->gikParam;
      gikParam.projectLink.resize(1);
      gikParam.projectLink[0] = moveContactConstraint->A_link() ? moveContactConstraint->A_link() : moveContactConstraint->B_link();
      gikParam.projectLocalPose = moveContactConstraint->A_link() ? moveContactConstraint->A_localpos() : moveContactConstraint->B_localpos();
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<contactCheckParam->variables.size();i++){
        if(contactCheckParam->variables[i]->isRevoluteJoint() || contactCheckParam->variables[i]->isPrismaticJoint()) {
          contactCheckParam->variables[i]->q() = std::max(std::min(contactCheckParam->variables[i]->q(),contactCheckParam->variables[i]->q_upper()),contactCheckParam->variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(contactCheckParam->variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          nominals,
                                                          gikParam,
                                                          tmpPath);
    }

    bool use_A_bodyContact = false;
    bool use_B_bodyContact = false;
    if ((ikState==IKState::ATTACH_PRE) && !solved) {
      for (int i=0; i<contactCheckParam->bodyContactConstraints.size() && !use_A_bodyContact && !use_B_bodyContact; i++) { // 現状は一方のみがbody contactを探索することが前提
        std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> bodyConstraint = std::static_pointer_cast<ik_constraint2_body_contact::BodyContactConstraint>(contactCheckParam->bodyContactConstraints[i]);
        if (bodyConstraint->A_link() == moveContactConstraint->A_link()) {
          use_A_bodyContact = true;
          // moveContactConstraint->A_localPos()
          // moveContactConstraint->B_localPos()
          moveContactConstraint->eval_link() = moveContactConstraint->B_link();
          moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
          moveContactConstraint->A_contact_pos_link() = bodyConstraint->A_contact_pos_link();
          moveContactConstraint->A_contact_pos_link()->T() = moveContactConstraint->A_localpos();
          moveContactConstraint->A_contactPoints() = bodyConstraint->A_contactPoints();
          moveContactConstraint->A_contactPointLength() = bodyConstraint->A_contactPointLength();
          moveContactConstraint->A_contactPointAreaDim() = bodyConstraint->A_contactPointAreaDim();
          moveContactConstraint->A_contactNormals() = bodyConstraint->A_contactNormals();
          moveContactConstraint->A_contactNormalJacobianXs() = bodyConstraint->A_contactNormalJacobianXs();
          moveContactConstraint->A_contactNormalJacobianYs() = bodyConstraint->A_contactNormalJacobianYs();
          moveContactConstraint->A_contactNormalJacobianZs() = bodyConstraint->A_contactNormalJacobianZs();
        }
        if (bodyConstraint->A_link() == moveContactConstraint->B_link()) {
          use_B_bodyContact = true;
          moveContactConstraint->A_localpos().linear() = moveContactConstraint->A_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.
          moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2); // 戻す
          if ((ikState==IKState::DETACH_FIXED) ||
              (ikState==IKState::ATTACH_PRE)) {
            if (moveContactConstraint->A_link()) moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.04);
            else moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.04);
            // 変えた分を戻す
            if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() -= moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
            else moveContactConstraint->B_localpos().translation() -= moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
          }
          moveContactConstraint->eval_link() = moveContactConstraint->A_link();
          moveContactConstraint->eval_localR() = moveContactConstraint->A_localpos().linear();
          moveContactConstraint->B_contact_pos_link() = bodyConstraint->A_contact_pos_link();
          moveContactConstraint->B_contact_pos_link()->T() = moveContactConstraint->B_localpos();
          moveContactConstraint->B_contactPoints() = bodyConstraint->A_contactPoints();
          moveContactConstraint->B_contactPointLength() = bodyConstraint->A_contactPointLength();
          moveContactConstraint->B_contactPointAreaDim() = bodyConstraint->A_contactPointAreaDim();
          moveContactConstraint->B_contactNormals() = bodyConstraint->A_contactNormals();
          moveContactConstraint->B_contactNormalJacobianXs() = bodyConstraint->A_contactNormalJacobianXs();
          moveContactConstraint->B_contactNormalJacobianYs() = bodyConstraint->A_contactNormalJacobianYs();
          moveContactConstraint->B_contactNormalJacobianZs() = bodyConstraint->A_contactNormalJacobianZs();
        }
        if (use_A_bodyContact && use_B_bodyContact) std::cerr << "[solveContactIK] error!! body contact searches in same link" << std::endl;
        if (use_A_bodyContact || use_B_bodyContact) {
          moveContactConstraint->weight() << bodyConstraint->weight();
          moveContactConstraint->contactSearchLimit() = bodyConstraint->contactSearchLimit();
          moveContactConstraint->precision() = bodyConstraint->precision();
          moveContactConstraint->contactWeight() = bodyConstraint->contactWeight();
          moveContactConstraint->normalGradientDistance() = bodyConstraint->normalGradientDistance();
          std::vector<cnoid::LinkPtr> variables = contactCheckParam->variables;
          variables.push_back(bodyConstraint->A_contact_pos_link());
          solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                         constraint,
                                                                         contactCheckParam->rejections,
                                                                         prevTasks,
                                                                         contactCheckParam->pikParam,
                                                                         tmpPath
                                                                         );
          if (solved) {
            for (int p=0; p<tmpPath->size(); p++) {
              (*tmpPath)[p].resize((*tmpPath)[p].size() - 7); // bodyContact用のframeを消す
            }
          }
        }
      }
    }

    if (solved && (ikState==IKState::DETACH_FIXED)) {
      postState.transition.insert(postState.transition.end(), (*tmpPath).begin(), (*tmpPath).end());
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint_nominal{constraints0, constraints1, nominals};
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks_;
      prioritized_inverse_kinematics_solver2::IKParam pikParam = contactCheckParam->pikParam;
      pikParam.minIteration = 40;
      prioritized_inverse_kinematics_solver2::solveIKLoop(contactCheckParam->variables,
                                                                     constraint_nominal,
                                                                     contactCheckParam->rejections,
                                                                     prevTasks_,
                                                                     pikParam,
                                                                     tmpPath
                                                                     );
    }

    // for ( int i=0; i<constraints0.size(); i++ ) {
    //   std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints1.size(); i++ ) {
    //   std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints2.size(); i++ ) {
    //   constraints2[i]->debugLevel() = 2;
    //   constraints2[i]->updateBounds();
    //   std::cerr << "constraints2: "<< constraints2[i]->isSatisfied() << std::endl;
    //   constraints2[i]->debugLevel() = 0;
    // }
    // for ( int i=0; i<nominals.size(); i++ ) {
    //   std::cerr << "nominals: "<< nominals[i]->isSatisfied() << std::endl;
    // }

    if (solved) {
      postState.transition.insert(postState.transition.end(), (*tmpPath).begin(), (*tmpPath).end());
      if (use_A_bodyContact) {
        moveContact.c1.localPose = moveContactConstraint->A_localpos();
        cnoid::Matrix3d B_rot = cnoid::Matrix3d::Identity();
        if (moveContactConstraint->B_link()) B_rot = moveContactConstraint->B_link()->R();
        cnoid::Matrix3d A_rot = moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear();
        moveContact.c2.localPose.linear() = (B_rot.transpose() * A_rot) * cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      } else if (use_B_bodyContact) {
        moveContact.c2.localPose = moveContactConstraint->B_localpos();
        cnoid::Matrix3d A_rot = cnoid::Matrix3d::Identity();
        if (moveContactConstraint->A_link()) A_rot = moveContactConstraint->A_link()->R();
        cnoid::Matrix3d B_rot = moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear();
        moveContact.c1.localPose.linear() = (A_rot.transpose() * B_rot) * cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      } else {
        moveContact.c1.localPose.linear() = moveContactConstraint->A_localpos().linear();
        cnoid::Matrix3d B_rot = cnoid::Matrix3d::Identity();
        if (moveContactConstraint->B_link()) B_rot = moveContactConstraint->B_link()->R();
        cnoid::Matrix3d A_rot;
        if (moveContactConstraint->A_link()) A_rot = moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear();
        else A_rot = moveContactConstraint->A_localpos().linear();
        moveContact.c2.localPose.linear() = (B_rot.transpose() * A_rot) * cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      }
    }

    return solved;

  }

  void convertParamFromCWCP(const cwcp::CWCPParam& param, const std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > >& cwcpPath, WholeBodyLocomotionContactPlanner& planner) {
    planner.bodies = param.bodies;
    planner.variables = param.variables;
    planner.constraints = param.constraints;
    planner.nominals = param.nominals;
    planner.currentContactState = std::make_shared<graph_search_wholebody_contact_planner::ContactState>();
    for (int i=0; i<param.currentContactPoints.size(); i++) {
      // c-wcpではc1がrobot, c2が環境(static)という順番
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      for (int b=0; b<param.bodies.size(); b++) {
        if (param.bodies[b]->link(param.currentContactPoints[i]->c1.link->name())) {
          c1.bodyName = param.bodies[b]->name();
          c1.linkName = param.currentContactPoints[i]->c1.link->name();
          c1.localPose = param.currentContactPoints[i]->c1.localPose;
          c1.isStatic = false;
        }
      }
      {
        if (param.currentContactPoints[i]->c2.link) {
          for (int b=0; b<param.bodies.size(); b++) {
            if (param.bodies[b]->link(param.currentContactPoints[i]->c2.link->name())) {
              c2.bodyName = param.bodies[b]->name();
              c2.linkName = param.currentContactPoints[i]->c2.link->name();
            }
          }
        }
        c2.localPose = param.currentContactPoints[i]->c2.localPose;
        c2.localPose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
        c2.isStatic = true;
      }
      planner.currentContactState->contacts.push_back(graph_search_wholebody_contact_planner::Contact(c1,c2));
    }
    planner.field = param.field;
    planner.guidePath.clear();
    planner.guidePath.resize(cwcpPath.size());
    for (int i=0; i<cwcpPath.size(); i++) {
      planner.guidePath[i].first = cwcpPath.at(i).first;
      for (int c=0; c<cwcpPath.at(i).second.size(); c++) {
        ContactCandidate c1;
        c1.bodyName = cwcpPath.at(i).second[c]->c1.link->body()->name();
        c1.linkName = cwcpPath.at(i).second[c]->c1.link->name();
        c1.localPose = cwcpPath.at(i).second[c]->c1.localPose;
        c1.isStatic = false;
        ContactCandidate c2;
        if (cwcpPath.at(i).second[c]->c2.link) {
          c2.bodyName = cwcpPath.at(i).second[c]->c2.link->body()->name();
          c2.linkName = cwcpPath.at(i).second[c]->c2.link->name();
        }
        c2.localPose = cwcpPath.at(i).second[c]->c2.localPose;
        c2.isStatic = true;
        planner.guidePath[i].second.push_back(Contact(c1, c2));
      }
    }
    planner.targetBodyName = param.projectLink->body()->name();
  }

  void convertDetachParamToCWCP(const WholeBodyContactPlanner& planner, const std::vector<Contact>& currentContacts, const cnoid::BodyPtr robot, const cnoid::BodyPtr object, const std::vector<std::shared_ptr<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint> > reachabilityConstraints, cwcp::CWCPParam& param, std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& removedConstraints, std::vector<Contact>& stableContacts) {
    param.bodies = planner.bodies;
    param.variables = planner.variables;
    param.nominals = planner.nominals;
    param.currentContactPoints.clear();
    removedConstraints.clear();
    param.constraints.clear();
    param.goals.clear();
    param.gikParam.projectLink.clear();
    // currentContactsについて
    // robotとobjectの接触をposition constraintに追加・stableContactsに追加
    // robotと、object以外の接触をcurrentContactsPointsに追加
    // robotに関与しない接触をstableContactsに追加
    std::vector<cnoid::LinkPtr> contactRobotLinks;
    std::vector<cnoid::LinkPtr> contactObjectLinks;
    for (int i=0; i<currentContacts.size(); i++) {
      if ((currentContacts[i].c1.bodyName == robot->name()) && (currentContacts[i].c2.bodyName == object->name())) {
        contactRobotLinks.push_back(robot->link(currentContacts[i].c1.linkName));
        contactObjectLinks.push_back(object->link(currentContacts[i].c2.linkName));
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link(currentContacts[i].c1.linkName);
        constraint->A_localpos() = currentContacts[i].c1.localPose;
        constraint->B_link() = object->link(currentContacts[i].c2.linkName);
        constraint->B_localpos() = constraint->B_link()->T().inverse() * constraint->A_link()->T() * constraint->A_localpos();
        param.constraints.push_back(constraint);
        stableContacts.push_back(currentContacts[i]);
      } else if ((currentContacts[i].c2.bodyName == robot->name()) && (currentContacts[i].c1.bodyName == object->name())) {
        contactRobotLinks.push_back(robot->link(currentContacts[i].c2.linkName));
        contactObjectLinks.push_back(object->link(currentContacts[i].c1.linkName));
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link(currentContacts[i].c2.linkName);
        constraint->A_localpos() = currentContacts[i].c2.localPose;
        constraint->B_link() = object->link(currentContacts[i].c1.linkName);
        constraint->B_localpos() = constraint->B_link()->T().inverse() * constraint->A_link()->T() * constraint->A_localpos();
        param.constraints.push_back(constraint);
        stableContacts.push_back(currentContacts[i]);
      } else if ((currentContacts[i].c1.bodyName == robot->name()) || (currentContacts[i].c2.bodyName == robot->name())) {
        std::shared_ptr<cwcp::Contact> contact = std::make_shared<cwcp::Contact>();
        if (currentContacts[i].c1.bodyName == robot->name()) {
          contact->c1.link = robot->link(currentContacts[i].c1.linkName);
          contact->c1.localPose = currentContacts[i].c1.localPose;
          for (int b=0; b<param.bodies.size(); b++) {
            if (param.bodies[b]->name() == currentContacts[i].c2.bodyName) {
              contact->c2.link = param.bodies[b]->link(currentContacts[i].c2.linkName);
              contact->c2.localPose = param.bodies[b]->link(currentContacts[i].c2.linkName)->T().inverse() * contact->c1.link->T() * contact->c1.localPose;
            } else {
              contact->c2.localPose = contact->c1.link->T() * contact->c1.localPose;
            }
          }
        }
        if (currentContacts[i].c2.bodyName == robot->name()) {
          contact->c1.link = robot->link(currentContacts[i].c2.linkName);
          contact->c1.localPose = currentContacts[i].c2.localPose;
          for (int b=0; b<param.bodies.size(); b++) {
            if (param.bodies[b]->name() == currentContacts[i].c1.bodyName) {
              contact->c2.link = param.bodies[b]->link(currentContacts[i].c1.linkName);
              contact->c2.localPose = param.bodies[b]->link(currentContacts[i].c1.linkName)->T().inverse() * contact->c1.link->T() * contact->c1.localPose;
            } else {
              contact->c2.localPose = contact->c1.link->T() * contact->c1.localPose;
            }
          }
        }
        param.currentContactPoints.push_back(contact);
      } else {
        stableContacts.push_back(currentContacts[i]);
      }
    }
    // contactしているlinkの干渉を無視
    for (int i=0; i<planner.constraints.size(); i++) {
      bool in_contact = false;
      if (typeid(*(planner.constraints[i]))==typeid(ik_constraint2_bullet::BulletCollisionConstraint)) {
        std::shared_ptr<ik_constraint2::CollisionConstraint> constraint = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(planner.constraints[i]);
        for (int l=0; l<contactRobotLinks.size() && !in_contact; l++) {
          if (((constraint->A_link() == contactRobotLinks[l]) && (constraint->B_link() == contactObjectLinks[l])) ||
              ((constraint->B_link() == contactRobotLinks[l]) && (constraint->A_link() == contactObjectLinks[l]))) {
            removedConstraints.push_back(constraint);
            in_contact = true;
          }
        }
      }
      if (!in_contact) param.constraints.push_back(planner.constraints[i]);
    }
    // 質量を修正
    param.reachabilityConstraints.clear();
    for (int i=0; i<reachabilityConstraints.size(); i++) {
      if (std::find(contactRobotLinks.begin(), contactRobotLinks.end(), reachabilityConstraints[i]->A_link()) == contactRobotLinks.end()) param.reachabilityConstraints.push_back(reachabilityConstraints[i]);
    }
  }

}
