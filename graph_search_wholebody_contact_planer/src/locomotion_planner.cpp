#include <graph_search_wholebody_contact_planner/locomotion_planner.h>
#include <ik_constraint2_scfr/ik_constraint2_scfr.h>
#include <limits>

namespace graph_search_wholebody_contact_planner{
  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyLocomotionContactPlanner::generateCheckParam() {
    std::shared_ptr<LocomotionContactTransitionCheckParam> checkParam = std::make_shared<LocomotionContactTransitionCheckParam>();
    cloneCheckParam(checkParam);
    return checkParam;
  }

  void WholeBodyLocomotionContactPlanner::cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) {
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    WholeBodyContactPlanner::cloneCheckParam(contactCheckParam);
    std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
    for(int b=0; b<this->bodies.size(); b++){
      modelMap[this->bodies[b]] = contactCheckParam->bodies[b];
    }
    contactCheckParam->bodyContactConstraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->bodyContactConstraints.size());
    for(int j=0;j<this->bodyContactConstraints.size();j++){
      contactCheckParam->bodyContactConstraints[j] = this->bodyContactConstraints[j]->clone(modelMap);
    }
    contactCheckParam->guidePath = this->guidePath;
  }

  bool WholeBodyLocomotionContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    ContactState state = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
    return true;
  }

  void WholeBodyLocomotionContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) {
    std::shared_ptr<LocomotionContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyLocomotionContactPlanner::LocomotionContactTransitionCheckParam>(checkParam);
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    if (state.transition.size() < 1) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.transition.size() < 1" << std::endl;
    if (state.transition[0].size() != state.frame.size()) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.transition[0].size() and state.frame.size() are mismatched!" << std::endl;
    if (state.frame.size() != contactCheckParam->guidePath[0].size()) std::cerr << "[WholeBodyLocomotionContactPlanner] error! state.frame.size() and contactCheckParam.guidePath[0].size() are mismatched!" << std::endl;
    int nearestIdx = -1;
    double diffMin = std::numeric_limits<double>::max();
    for (int i=1; i<contactCheckParam->guidePath.size(); i++) {
      double diff = 0;
      unsigned int idx=0;
      for (int l=0; l<contactCheckParam->variables.size(); l++) {
        if (contactCheckParam->variables[l]->isRevoluteJoint() || contactCheckParam->variables[l]->isPrismaticJoint()) {
          diff += std::abs(state.frame[idx] - contactCheckParam->guidePath[i][idx]);
          idx+=1;
        } else if (contactCheckParam->variables[l]->isFreeJoint()) {
          cnoid::Vector3 prePos = cnoid::Vector3(state.transition[0][idx+0], state.transition[0][idx+1], state.transition[0][idx+2]);
          cnoid::Vector3 postPos = cnoid::Vector3(state.frame[idx+0], state.frame[idx+1], state.frame[idx+2]);
          cnoid::Vector3 guidePrePos = cnoid::Vector3(contactCheckParam->guidePath[i-1][idx+0], contactCheckParam->guidePath[i-1][idx+1], contactCheckParam->guidePath[i-1][idx+2]);
          cnoid::Vector3 guidePostPos = cnoid::Vector3(contactCheckParam->guidePath[i][idx+0], contactCheckParam->guidePath[i][idx+1], contactCheckParam->guidePath[i][idx+2]);
          diff -= (postPos - prePos).dot(guidePostPos - guidePrePos);
          diff += std::abs(cnoid::AngleAxis(cnoid::Quaternion(state.frame[idx+6],state.frame[idx+3],state.frame[idx+4],state.frame[idx+5]).toRotationMatrix().transpose() * cnoid::Quaternion(contactCheckParam->guidePath[i][idx+6],contactCheckParam->guidePath[i][idx+3],contactCheckParam->guidePath[i][idx+4],contactCheckParam->guidePath[i][idx+5]).toRotationMatrix()).angle());
          idx+=7;
        }
      }
      if (diff < diffMin) {
        diffMin = diff;
        nearestIdx = i;
      }
    }
    node->heuristic() = (contactCheckParam->guidePath.size() - nearestIdx) * 1e4 + diffMin;
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
      std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
      scfrConstraint->A_robot() = contactCheckParam->bodies[b];
      scfrConstraints.push_back(scfrConstraint);
    }

    {
      for (int i=0; i<contactCheckParam->preState.contacts.size(); i++) {
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        if (contactCheckParam->preState.contacts[i].c1.isStatic) { constraint->A_link() = nullptr; }
        else {
          for (int b=0; b<contactCheckParam->bodies.size(); b++) {
            if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c1.bodyName) continue;
            if (contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c1.linkName)) {
              constraint->A_link() = contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c1.linkName);
              break;
            }
          }
          if (!constraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have preState.contacts[i].c1.linkName" << std::endl;
        }
        constraint->A_localpos() = contactCheckParam->preState.contacts[i].c1.localPose;
        if (contactCheckParam->preState.contacts[i].c2.isStatic) { constraint->B_link() = nullptr; }
        else {
          for (int b=0; b<contactCheckParam->bodies.size(); b++) {
            if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) continue;
            if (contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c2.linkName)) {
              constraint->B_link() = contactCheckParam->bodies[b]->joint(contactCheckParam->preState.contacts[i].c2.linkName);
              break;
            }
          }
          if (!constraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have preState.contacts[i].c2.linkName" << std::endl;
        }
        constraint->B_localpos() = contactCheckParam->preState.contacts[i].c2.localPose;
        constraint->B_localpos().linear() = constraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるためにZの向きを揃える.
        constraint->eval_link() = constraint->B_link();
        constraint->eval_localR() = constraint->B_localpos().linear();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
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
    if (moveContact.c1.isStatic) { moveContactConstraint->A_link() = nullptr; }
    else {
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if (contactCheckParam->bodies[b]->name() != moveContact.c1.bodyName) continue;
        if (contactCheckParam->bodies[b]->joint(moveContact.c1.linkName)) {
          moveContactConstraint->A_link() = contactCheckParam->bodies[b]->joint(moveContact.c1.linkName);
          break;
        }
      }
      if (!moveContactConstraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have postState.contacts[i].c1.linkName" << std::endl;
    }
    moveContactConstraint->A_localpos() = moveContact.c1.localPose;
    if (moveContact.c2.isStatic) { moveContactConstraint->B_link() = nullptr; }
    else {
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if (contactCheckParam->bodies[b]->name() != moveContact.c2.bodyName) continue;
        if (contactCheckParam->bodies[b]->joint(moveContact.c2.linkName)) {
          moveContactConstraint->B_link() = contactCheckParam->bodies[b]->joint(moveContact.c2.linkName);
          break;
        }
      }
      if (!moveContactConstraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have postState.contacts[i].c2.linkName" << std::endl;
    }
    moveContactConstraint->B_localpos() = moveContact.c2.localPose;
    moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.

    if ((ikState==IKState::DETACH_FIXED) ||
        (ikState==IKState::ATTACH_PRE)) {
      if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
      else moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
    }
    moveContactConstraint->eval_link() = moveContactConstraint->B_link();
    moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
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

    bool use_A_bodyContact = false;
    bool use_B_bodyContact = false;
    if (!solved) {
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
            if (moveContactConstraint->A_link()) moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.02);
            else moveContactConstraint->A_localpos().translation() += moveContactConstraint->A_localpos().linear() * cnoid::Vector3(0,0,0.02);
            // 変えた分を戻す
            if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() -= moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
            else moveContactConstraint->B_localpos().translation() -= moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
          }
          moveContactConstraint->eval_link() = moveContactConstraint->A_link();
          moveContactConstraint->eval_localR() = moveContactConstraint->A_localpos().linear();
          moveContactConstraint->B_contact_pos_link() = bodyConstraint->A_contact_pos_link();
          moveContactConstraint->B_contact_pos_link()->T() = moveContactConstraint->A_localpos();
          moveContactConstraint->B_contactPoints() = bodyConstraint->A_contactPoints();
          moveContactConstraint->B_contactNormals() = bodyConstraint->A_contactNormals();
          moveContactConstraint->B_contactNormalJacobianXs() = bodyConstraint->A_contactNormalJacobianXs();
          moveContactConstraint->B_contactNormalJacobianYs() = bodyConstraint->A_contactNormalJacobianYs();
          moveContactConstraint->B_contactNormalJacobianZs() = bodyConstraint->A_contactNormalJacobianZs();
        }
        if (use_A_bodyContact && use_B_bodyContact) std::cerr << "[solveContactIK] error!! body contact searches in same link" << std::endl;
        if (use_A_bodyContact || use_B_bodyContact) {
          moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
          moveContactConstraint->contactSearchLimit() = bodyConstraint->contactSearchLimit();
          moveContactConstraint->precision() = bodyConstraint->precision();
          moveContactConstraint->contactWeight() = bodyConstraint->contactWeight();
          moveContactConstraint->normalGradientDistance() = bodyConstraint->normalGradientDistance();
          solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(contactCheckParam->variables,
                                                                         constraint,
                                                                         contactCheckParam->rejections,
                                                                         prevTasks,
                                                                         contactCheckParam->pikParam,
                                                                         tmpPath
                                                                         );
        }
      }
    }

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
}
