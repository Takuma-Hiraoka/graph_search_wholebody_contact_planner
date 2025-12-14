#include <graph_search_wholebody_contact_planner/manipulation_planner.h>
#include <ik_constraint2_scfr/ik_constraint2_scfr.h>
#include <limits>
#include <random>
#include <set>

namespace graph_search_wholebody_contact_planner{
  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyManipulationContactPlanner::generateCheckParam() {
    std::shared_ptr<ManipulationContactTransitionCheckParam> checkParam = std::make_shared<ManipulationContactTransitionCheckParam>();
    cloneCheckParam(checkParam);
    return checkParam;
  }

  void WholeBodyManipulationContactPlanner::cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) {
    std::shared_ptr<ManipulationContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam>(checkParam);
    WholeBodyContactPlanner::cloneCheckParam(contactCheckParam);
    contactCheckParam->targetContact = this->targetContact;
  }

  bool WholeBodyManipulationContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    ContactState state = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
    std::shared_ptr<ManipulationContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam>(checkParam);
    if (contactCheckParam->targetContact.second) { // attach
      for (int i=0; i<state.contacts.size(); i++) {
        if (state.contacts[i] == contactCheckParam->targetContact.first) return true;
      }
      return false;
    } else { // detach
      for (int i=0; i<state.contacts.size(); i++) {
        if (state.contacts[i] == contactCheckParam->targetContact.first) return false;
      }
      return true;
    }
  }

  void WholeBodyManipulationContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) {
    std::shared_ptr<ManipulationContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam>(checkParam);
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    double heuristic = 0;
    double nSatisfy = 1e3;
    double contactWeight = 1e1;
    double scfrNSatisfy = 1e2;
    if (contactCheckParam->targetContact.second) { // attach
      bool attach = false;
      for (int i=0; i<state.contacts.size(); i++) {
        if (state.contacts[i] == contactCheckParam->targetContact.first) attach = true;
      }
      if (!attach) heuristic += nSatisfy;
    } else { // detach
      bool detach = true;
      for (int i=0; i<state.contacts.size(); i++) {
        if (state.contacts[i] == contactCheckParam->targetContact.first) detach = false;
      }
      if (!detach) {
        heuristic += nSatisfy;
        heuristic += state.contacts.size() * contactWeight;
        // 対象物体にはひとつは触れているように
        bool in_contact = false;
        for (int i=0; i<state.contacts.size(); i++) {
          if (state.contacts[i] == contactCheckParam->targetContact.first) continue;
          if (!contactCheckParam->targetContact.first.c1.isStatic &&
              ((state.contacts[i].c1.bodyName == contactCheckParam->targetContact.first.c1.bodyName) ||
               (state.contacts[i].c2.bodyName == contactCheckParam->targetContact.first.c1.bodyName))
              ) in_contact = true;
          else if (!contactCheckParam->targetContact.first.c2.isStatic &&
              ((state.contacts[i].c1.bodyName == contactCheckParam->targetContact.first.c2.bodyName) ||
               (state.contacts[i].c2.bodyName == contactCheckParam->targetContact.first.c2.bodyName))
              ) in_contact = true;
        }
        if (in_contact) heuristic -= contactWeight;
        // detachが可能になる（SCFRが存在するようになる）ときに良く
        // targetContactに関係するbodyのみ
        // targetContactに関係するbodyは既にfixed bodyとkinematics tree上で連結しているという仮定
        // すなわち、targetContact側のbody及びlinkは不動
        std::vector<Contact> checkContacts = state.contacts;
        checkContacts.erase(std::remove(checkContacts.begin(), checkContacts.end(), contactCheckParam->targetContact.first), checkContacts.end());
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
        // targetContact.first.c1
        if (!targetContact.first.c1.isStatic) {
          bool solved = false;
          for (int b=0; b<contactCheckParam->bodies.size(); b++) {
            if (contactCheckParam->bodies[b]->name() == targetContact.first.c1.bodyName) {
              if (!contactCheckParam->bodies[b]->link(targetContact.first.c1.linkName)) {
                std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have targetContact.first.c1.linkName !!" << std::endl;
                continue;
              }
              std::vector<cnoid::Isometry3> poses;
              std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
              std::vector<cnoid::VectorX> bs;
              std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
              std::vector<cnoid::VectorX> dls;
              std::vector<cnoid::VectorX> dus;
              for (int c=0; c<checkContacts.size(); c++) {
                if (checkContacts[c].c1.bodyName == targetContact.first.c1.bodyName) {
                  if (contactCheckParam->bodies[b]->link(checkContacts[c].c1.linkName)) {
                    poses.push_back(contactCheckParam->bodies[b]->link(checkContacts[c].c1.linkName)->T() * checkContacts[c].c1.localPose);
                    As.emplace_back(0,6);
                    bs.emplace_back(0);
                    Cs.push_back(C);
                    dls.push_back(dl);
                    dus.push_back(du);
                  } else std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have checkContacts[c].c1.linkName !!" << std::endl;
                } else if (checkContacts[c].c2.bodyName == targetContact.first.c1.bodyName) {
                  if (contactCheckParam->bodies[b]->link(checkContacts[c].c2.linkName)) {
                    poses.push_back(contactCheckParam->bodies[b]->link(checkContacts[c].c2.linkName)->T() * checkContacts[c].c2.localPose);
                    As.emplace_back(0,6);
                    bs.emplace_back(0);
                    Cs.push_back(C);
                    dls.push_back(dl);
                    dus.push_back(du);
                  } else std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have checkContacts[c].c2.linkName !!" << std::endl;
                }
              }
              scfr_solver::SCFRParam scfrParam;
              if (poses.size() <= 1) continue; // エッジに触れるだけでscfrができることがあるが、その後の持ち上げができないことが多いため、1点のみの接触は不可
              solved = ik_constraint2_keep_collision_scfr::checkSCFRExistance(poses,
                                                                              As,
                                                                              bs,
                                                                              Cs,
                                                                              dls,
                                                                              dus,
                                                                              contactCheckParam->bodies[b]->mass(),
                                                                              scfrParam
                                                                              );
            }
          }
          if (!solved) heuristic += scfrNSatisfy;
        }
        // targetContact.first.c2
        if (!targetContact.first.c2.isStatic) {
          bool solved = false;
          for (int b=0; b<contactCheckParam->bodies.size(); b++) {
            if (contactCheckParam->bodies[b]->name() == targetContact.first.c2.bodyName) {
              if (!contactCheckParam->bodies[b]->link(targetContact.first.c2.linkName)) {
                std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have targetContact.first.c2.linkName !!" << std::endl;
                continue;
              }
              std::vector<cnoid::Isometry3> poses;
              std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
              std::vector<cnoid::VectorX> bs;
              std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
              std::vector<cnoid::VectorX> dls;
              std::vector<cnoid::VectorX> dus;
              for (int c=0; c<checkContacts.size(); c++) {
                if (checkContacts[c].c1.bodyName == targetContact.first.c2.bodyName) {
                  if (contactCheckParam->bodies[b]->link(checkContacts[c].c1.linkName)) {
                    poses.push_back(contactCheckParam->bodies[b]->link(checkContacts[c].c1.linkName)->T() * checkContacts[c].c1.localPose);
                    As.emplace_back(0,6);
                    bs.emplace_back(0);
                    Cs.push_back(C);
                    dls.push_back(dl);
                    dus.push_back(du);
                  } else std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have checkContacts[c].c1.linkName !!" << std::endl;
                } else if (checkContacts[c].c2.bodyName == targetContact.first.c2.bodyName) {
                  if (contactCheckParam->bodies[b]->link(checkContacts[c].c2.linkName)) {
                    poses.push_back(contactCheckParam->bodies[b]->link(checkContacts[c].c2.linkName)->T() * checkContacts[c].c2.localPose);
                    As.emplace_back(0,6);
                    bs.emplace_back(0);
                    Cs.push_back(C);
                    dls.push_back(dl);
                    dus.push_back(du);
                  } else std::cerr << "[WholeBodyManipulationContactPlanner] contactCheckParam->bodies[b] does not have checkContacts[c].c2.linkName !!" << std::endl;
                }
              }
              scfr_solver::SCFRParam scfrParam;
              if (poses.size() <= 1) continue; // エッジに触れるだけでscfrができることがあるが、その後の持ち上げができないことが多いため、1点のみの接触は不可
              solved = ik_constraint2_keep_collision_scfr::checkSCFRExistance(poses,
                                                                              As,
                                                                              bs,
                                                                              Cs,
                                                                              dls,
                                                                              dus,
                                                                              contactCheckParam->bodies[b]->mass(),
                                                                              scfrParam
                                                                              );
            }
          }
          if (!solved) heuristic += scfrNSatisfy;
        }

      }
    }
    // std::cerr << "heuristic " << heuristic << std::endl;
    node->heuristic() = heuristic;
  }

  std::vector<std::shared_ptr<graph_search::Node> > WholeBodyManipulationContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ManipulationContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam>(checkParam);
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
      for (int j=0; j<contactCheckParam->contactStaticCandidates.size(); j++) {
        if ((rootPos - contactCheckParam->contactStaticCandidates[j]->localPose.translation()).norm() > contactCheckParam->addCandidateDistance) continue;

        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->state() = extend_state;
        Contact c = Contact(*(contactCheckParam->contactDynamicCandidates[i]), *(contactCheckParam->contactStaticCandidates[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    std::set<std::string> contactLinkOfRobot;
    for (int i=0; i<contactCheckParam->contactDynamicCandidates.size(); i++) {
      if (contactCheckParam->contactDynamicCandidates[i]->bodyName == contactCheckParam->bodies[0]->name()) contactLinkOfRobot.insert(contactCheckParam->contactDynamicCandidates[i]->linkName);
    }
    int numContactLink = contactLinkOfRobot.size();
    int numObjectContact = 0;
    for (int i=0; i<extend_state.contacts.size(); i++) {
      if (!extend_state.contacts[i].c1.isStatic && !extend_state.contacts[i].c2.isStatic) numObjectContact++; // !isStatic同士の接触はrobotとobjectだけという仮定がある.
    }
    // dynamic contact
    std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidatesBuf;
    for (int i=0; i<contactCheckParam->contactDynamicCandidates.size(); i++) {
      // 既に接触している接触候補は接触できない
      bool in_contact = false;
      for (int j=0; j<extend_state.contacts.size() && !in_contact; j++) {
        if (((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c1.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c1.linkName) && (contactCheckParam->contactDynamicCandidates[i]->localPose.translation() == extend_state.contacts[j].c1.localPose.translation())) ||
            ((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c2.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c2.linkName) && (contactCheckParam->contactDynamicCandidates[i]->localPose.translation() == extend_state.contacts[j].c2.localPose.translation()))) in_contact = true;
      }
      if (!in_contact) contactDynamicCandidatesBuf.push_back(contactCheckParam->contactDynamicCandidates[i]);
    }

    for (int i=0; i<contactDynamicCandidatesBuf.size(); i++) {
      // それぞれのルートリンク位置の距離がaddCandidateDistanceを超えるcontactDynamicCandidate同士を接触させることはしない
      // 高速化のため. gikを使うまでもなく解けない
      cnoid::Vector3 rootPos1;
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if ((contactCheckParam->bodies[b]->name() == contactDynamicCandidatesBuf[i]->bodyName) && contactCheckParam->bodies[b]->joint(contactDynamicCandidatesBuf[i]->linkName)) rootPos1 = contactCheckParam->bodies[b]->rootLink()->p();
      }
      for (int j=i+1; j<contactDynamicCandidatesBuf.size(); j++) {
        // 同じBody内の候補同士は接触できない
        // TODO Link内にする?
        if ((contactDynamicCandidatesBuf[i]->bodyName == contactDynamicCandidatesBuf[j]->bodyName)/* && (contactDynamicCandidatesBuf[i]->linkName == contactDynamicCandidatesBuf[j]->linkName)*/) continue;
        // robotの接触候補link数 - robotとobjectとの接触 が2以上になるようにする. その後動いたりするため
        // -->robotとobjectとの接触はrobotの接触候補link数 - robotとobjectとの接触が2以下のときは新たにrobotとobjectの接触を追加しない
        if ((((contactDynamicCandidatesBuf[i]->bodyName == contactCheckParam->bodies[0]->name()) && !contactDynamicCandidatesBuf[j]->isStatic) ||
             ((contactDynamicCandidatesBuf[j]->bodyName == contactCheckParam->bodies[0]->name()) && !contactDynamicCandidatesBuf[i]->isStatic)) &&
            (numContactLink - numObjectContact <= 2)) continue;
        cnoid::Vector3 rootPos2;
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if ((contactCheckParam->bodies[b]->name() == contactDynamicCandidatesBuf[j]->bodyName) && contactCheckParam->bodies[b]->joint(contactDynamicCandidatesBuf[j]->linkName)) rootPos2 = contactCheckParam->bodies[b]->rootLink()->p();
        }
        if ((rootPos1 - rootPos2).norm() > contactCheckParam->addCandidateDistance) continue;

        // localPoseが違ったとしても既に接触しているリンク同士を更に接触させることはしない
        bool found = false;
        for (int k=0; k<extend_state.contacts.size() && !found; k++) {
          if (extend_state.contacts[k] == Contact(*(contactDynamicCandidatesBuf[i]), *(contactDynamicCandidatesBuf[j]))) found = true;
        }
        if (found) continue;

        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        newNode->state() = extend_state;
        Contact c = Contact(*(contactDynamicCandidatesBuf[i]), *(contactDynamicCandidatesBuf[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(adjacentNodes.begin(), adjacentNodes.end(), g);
    return adjacentNodes;

  }

  bool WholeBodyManipulationContactPlanner::solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                                                         Contact& moveContact,
                                                         ContactState& postState,
                                                         const IKState& ikState
                                                         ) {
    std::shared_ptr<const WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<const WholeBodyManipulationContactPlanner::ManipulationContactTransitionCheckParam>(checkParam);
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
          if (contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c1.linkName)) {
            constraint->A_link() = contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c1.linkName);
            break;
          }
        }
        constraint->A_localpos() = contactCheckParam->preState.contacts[i].c1.localPose;
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) continue;
          if (contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c2.linkName)) {
            constraint->B_link() = contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c2.linkName);
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
          // robotのSCFRは、objectとの接触を除いて調べる
          // objectの質量はここでは無視. robotの質量に対して十分小さい
          if ((scfrConstraints[j]->A_robot() == contactCheckParam->bodies[0]) && // robotについて
              (((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c1.bodyName) && !contactCheckParam->preState.contacts[i].c2.isStatic) ||
               ((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c2.bodyName) && !contactCheckParam->preState.contacts[i].c1.isStatic)))
            {
              continue;
          }
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
    bool use_A_robot = false;
    bool use_B_robot = false;
    for (int i=0; i<contactCheckParam->bodyContactConstraints.size() && !use_A_robot && !use_B_robot; i++) {
      std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> bodyConstraint = std::static_pointer_cast<ik_constraint2_body_contact::BodyContactConstraint>(contactCheckParam->bodyContactConstraints[i]);
      if (bodyConstraint->A_link() == moveContactConstraint->A_link()) use_A_robot = true;
      if (bodyConstraint->A_link() == moveContactConstraint->B_link()) use_B_robot = true;
    }
    if (use_A_robot || !use_B_robot) {
      moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.
      if ((ikState==IKState::DETACH_FIXED) ||
          (ikState==IKState::ATTACH_PRE)) {
        if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.1);
        else moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.1);
      }
      moveContactConstraint->eval_link() = moveContactConstraint->B_link();
      moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
    }else if (use_B_robot) {
      moveContactConstraint->A_localpos().linear() = moveContactConstraint->A_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.
      if ((ikState==IKState::DETACH_FIXED) ||
          (ikState==IKState::ATTACH_PRE)) {
        if (moveContactConstraint->A_link()) moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.1);
        else moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.1);
      }
      moveContactConstraint->eval_link() = moveContactConstraint->A_link();
      moveContactConstraint->eval_localR() = moveContactConstraint->A_localpos().linear();
    }

    moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
    constraints2.push_back(moveContactConstraint);

    for (int i=0;i<scfrConstraints.size();i++) {
      if (scfrConstraints[i]->poses().size() == 0) return false; // 接触が存在しない物体がある.
      constraints0.push_back(scfrConstraints[i]);
    }

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint{constraints0, constraints1, constraints2, contactCheckParam->nominals};

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
                                                          contactCheckParam->nominals,
                                                          gikParam,
                                                          tmpPath);
    }

    if ((ikState==IKState::ATTACH_PRE) && !solved) {
      bool use_A_bodyContact = false;
      bool use_B_bodyContact = false;
      for (int i=0; i<contactCheckParam->bodyContactConstraints.size() && !use_A_bodyContact && !use_B_bodyContact; i++) { // 現状は一方のみがbody contactを探索することが前提
        std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> bodyConstraint = std::static_pointer_cast<ik_constraint2_body_contact::BodyContactConstraint>(contactCheckParam->bodyContactConstraints[i]);
        if (bodyConstraint->A_link() == moveContactConstraint->A_link()) {
          use_A_bodyContact = true;
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
            solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                                gikConstraints,
                                                                constraints2,
                                                                contactCheckParam->nominals,
                                                                gikParam,
                                                                tmpPath);
          }
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
      if (use_A_robot) {
        moveContact.c1.localPose = moveContactConstraint->A_localpos();
        cnoid::Matrix3d B_rot = cnoid::Matrix3d::Identity();
        if (moveContactConstraint->B_link()) B_rot = moveContactConstraint->B_link()->R();
        cnoid::Matrix3d A_rot = moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear();
        moveContact.c2.localPose.linear() = (B_rot.transpose() * A_rot) * cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      } else if (use_B_robot) {
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

}
