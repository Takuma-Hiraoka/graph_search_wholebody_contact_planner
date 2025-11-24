#include <choreonoid_viewer/choreonoid_viewer.h>

#include <graph_search_wholebody_contact_planner/locomotion_planner.h>
#include <graph_search_wholebody_contact_planner/manipulation_planner.h>
#include <choreonoid_contact_candidate_generator/choreonoid_contact_candidate_generator.h>
#include "jaxon_common.h"
#include "world_common.h"

namespace graph_search_wholebody_contact_planner_sample{
  void sample2_locomanipulation(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<cwcp::CWCPParam> param = std::make_shared<cwcp::CWCPParam>();
    generateHallWorld(obstacle, param);

    cnoid::BodyPtr cube;
    generateCUBE(cube, param);

    cnoid::BodyPtr robot;
    generateJAXON(robot, param);

    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);
    // goal
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->B_localpos() = cube->rootLink()->T();
      constraint->weight() << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
      constraint->precision() = 0.4;
      param->goals.push_back(constraint);
    }

    param->gikParam.range = 0.5;
    param->gikParam.delta = 0.45;
    param->gikParam.goalBias = 0.8;
    param->gikParam.timeout = 60;
    param->gikParam.threads = 20;
    param->OptimizeTrajectory = true;
    param->toParam.initialShortcut = true;
    param->toParam.shortcut = true;
    param->toParam.shortcutThre = param->gikParam.delta; // 小さくしすぎるとIKが解けない確率が増える

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    for(int b=0; b<param->bodies.size(); b++) viewer->objects(param->bodies[b]);
    viewer->drawObjects();

    // param->pikParam.debugLevel = 3;
    // param->pikParam.viewMilliseconds = -1;
    // param->pikParam.viewer = viewer;

    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > > cwcpPath;
    if(!cwcp::solveCWCP(param, cwcpPath)) std::cerr << "solveCWCP failed" << std::endl;
    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > > keyPosePath;
    if(!cwcp::generateKeyPose(param, cwcpPath, keyPosePath)) std::cerr << "generateKeyPose failed" << std::endl;

    global_inverse_kinematics_solver::frame2Link(initialPose, param->variables);

    graph_search_wholebody_contact_planner::WholeBodyLocomotionContactPlanner locoPlanner;
    graph_search_wholebody_contact_planner::convertCWCPParam(*param, keyPosePath, locoPlanner);
    global_inverse_kinematics_solver::link2Frame(locoPlanner.variables, locoPlanner.currentContactState->frame);
    // planner.rejections
    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> csc_;
    choreonoid_contact_candidate_generator::generateCC(obstacle, csc_, 0.2);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > environmentContacts;
    graph_search_wholebody_contact_planner::convertContactCandidates(csc_, environmentContacts, true);

    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> cube_;
    choreonoid_contact_candidate_generator::generateCC(cube, cube_, 0.1);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > cubeContacts;
    graph_search_wholebody_contact_planner::convertContactCandidates(cube_, cubeContacts, true);
    locoPlanner.contactStaticCandidates.insert(locoPlanner.contactStaticCandidates.end(), environmentContacts.begin(), environmentContacts.end());
    locoPlanner.contactStaticCandidates.insert(locoPlanner.contactStaticCandidates.end(), cubeContacts.begin(), cubeContacts.end());

    addLimbInfo(locoPlanner, robot);

    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > larmGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "LARM_JOINT7", larmGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > rarmGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "RARM_JOINT7", rarmGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > llegGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "LLEG_JOINT5", llegGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > rlegGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "RLEG_JOINT5", rlegGuidedCandidates);

    {
      std::vector<cnoid::SgNodePtr> drawOnObjects;
      cnoid::BodyPtr ccMarkers = new cnoid::Body();
      {
        cnoid::LinkPtr rootLink = new cnoid::Link();
        ccMarkers->setRootLink(rootLink);
        graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, larmGuidedCandidates, cnoid::Vector3f(1.0, 0.0, 0.0));
        graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, rarmGuidedCandidates, cnoid::Vector3f(0.0, 0.0, 1.0));
        graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, llegGuidedCandidates, cnoid::Vector3f(0.5, 0.5, 0.0));
        graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, rlegGuidedCandidates, cnoid::Vector3f(0.0, 0.5, 0.5));
      }
      drawOnObjects.push_back(ccMarkers->rootLink()->visualShape());
      viewer->drawOn(drawOnObjects);
    }

    viewer->drawObjects();

    // locoPlanner.pikParam.debugLevel = 3;
    // locoPlanner.pikParam.viewMilliseconds = -1;
    // locoPlanner.pikParam.viewer = viewer;

    locoPlanner.addCandidateDistance = 1.5;
    locoPlanner.threads() = 10;
    locoPlanner.debugLevel() = 0;
    // locoPlanner.maxExtendNum() = 1000;

    locoPlanner.solve();

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsGoPath;
    locoPlanner.goalPath(gsGoPath);

    std::cerr << "Go to cube OK" << std::endl;
    global_inverse_kinematics_solver::frame2Link(gsGoPath.back().frame, locoPlanner.variables);
    for(int b=0; b<param->bodies.size(); b++) {
      param->bodies[b]->calcForwardKinematics(false);
      param->bodies[b]->calcCenterOfMass();
    }

    // detach
    graph_search_wholebody_contact_planner::WholeBodyManipulationContactPlanner manipPlanner;
    manipPlanner.bodies = locoPlanner.bodies;
    manipPlanner.variables = locoPlanner.variables;
    manipPlanner.constraints = locoPlanner.constraints;
    manipPlanner.rejections = locoPlanner.rejections;
    manipPlanner.nominals = locoPlanner.nominals;
    cube->rootLink()->setJointType(cnoid::Link::JointType::FreeJoint);
    manipPlanner.variables.push_back(cube->rootLink());
    manipPlanner.threads() = locoPlanner.threads();
    manipPlanner.addCandidateDistance = 1.2;
    manipPlanner.debugLevel() = 0;
    manipPlanner.currentContactState = std::make_shared<graph_search_wholebody_contact_planner::ContactState>(gsGoPath.back());
    for (int i=0; i<manipPlanner.currentContactState->contacts.size(); i++) {
      if (manipPlanner.currentContactState->contacts[i].c1.bodyName == cube->name() && manipPlanner.currentContactState->contacts[i].c1.linkName == "cube") manipPlanner.currentContactState->contacts[i].c1.isStatic = false;
      if (manipPlanner.currentContactState->contacts[i].c2.bodyName == cube->name() && manipPlanner.currentContactState->contacts[i].c2.linkName == "cube") manipPlanner.currentContactState->contacts[i].c2.isStatic = false;
    }
    {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      c1.bodyName = cube->name();
      c1.linkName = "cube";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, -0.2);
      c1.localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, 0.0);
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      c2.bodyName = "floor";
      c2.linkName = "table2";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(0.7, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      manipPlanner.currentContactState->contacts.push_back(graph_search_wholebody_contact_planner::Contact(c1,c2));
    }
    global_inverse_kinematics_solver::link2Frame(manipPlanner.variables, manipPlanner.currentContactState->frame);
    manipPlanner.contactStaticCandidates.insert(manipPlanner.contactStaticCandidates.end(), environmentContacts.begin(), environmentContacts.end());
    for (int i=0; i<cubeContacts.size(); i++) cubeContacts[i]->isStatic = false;
    manipPlanner.contactDynamicCandidates.insert(manipPlanner.contactDynamicCandidates.end(), cubeContacts.begin(), cubeContacts.end());
    addLimbInfo(manipPlanner, robot);

    // 現在触れているかどうかの判定にはlocalPoseを用いるが、currentContactで接触点探索が行われている場合localPoseが変わっている。このため、currentContactに含まれる、ロボットのDynamicCandidateのlocalPoseはcurrentContactのものにする
    for (int i=0; i<manipPlanner.currentContactState->contacts.size(); i++) {
      if(manipPlanner.currentContactState->contacts[i].c1.bodyName == robot->name()) {
        for (int j=0; j<manipPlanner.contactDynamicCandidates.size(); j++) {
          if ((manipPlanner.contactDynamicCandidates[j]->bodyName == manipPlanner.currentContactState->contacts[i].c1.bodyName) && (manipPlanner.contactDynamicCandidates[j]->linkName == manipPlanner.currentContactState->contacts[i].c1.linkName)) {
            manipPlanner.contactDynamicCandidates[j]->localPose = manipPlanner.currentContactState->contacts[i].c1.localPose;
          }
        }
      }
      if(manipPlanner.currentContactState->contacts[i].c2.bodyName == robot->name()) {
        for (int j=0; j<manipPlanner.contactDynamicCandidates.size(); j++) {
          if ((manipPlanner.contactDynamicCandidates[j]->bodyName == manipPlanner.currentContactState->contacts[i].c2.bodyName) && (manipPlanner.contactDynamicCandidates[j]->linkName == manipPlanner.currentContactState->contacts[i].c2.linkName)) {
            manipPlanner.contactDynamicCandidates[j]->localPose = manipPlanner.currentContactState->contacts[i].c2.localPose;
          }
        }
      }
    }
    manipPlanner.field = locoPlanner.field;
    // targetContact
    {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      c1.bodyName = cube->name();
      c1.linkName = "cube";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, -0.2);
      c1.localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, 0.0);
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      c2.bodyName = "floor";
      c2.linkName = "table2";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(0.7, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      manipPlanner.targetContact = std::make_pair(graph_search_wholebody_contact_planner::Contact(c1,c2), false);
    }

    {
      std::vector<cnoid::SgNodePtr> drawOnObjects;
      std::vector<cnoid::SgNodePtr> csc = graph_search_wholebody_contact_planner::generateCandidateMarkers(manipPlanner.bodies, manipPlanner.contactStaticCandidates);
      std::vector<cnoid::SgNodePtr> cdc = graph_search_wholebody_contact_planner::generateCandidateMarkers(manipPlanner.bodies, manipPlanner.contactDynamicCandidates);
      drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
      drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());
      viewer->drawOn(drawOnObjects);
    }
    viewer->drawObjects();

    if (manipPlanner.solve()) std::cerr << "detach solved" << std::endl;
    else std::cerr << "detach failed" << std::endl;

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsDetachPath;
    manipPlanner.goalPath(gsDetachPath);

    while (true) {
      for(int i=0;i<cwcpPath.size();i++){
        global_inverse_kinematics_solver::frame2Link(cwcpPath.at(i).first,param->variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      for(int i=0;i<keyPosePath.size();i++){
        global_inverse_kinematics_solver::frame2Link(keyPosePath.at(i).first,param->variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      for(int i=0;i<gsGoPath.size();i++){
        for (int j=0;j<gsGoPath[i].transition.size();j++) {
          global_inverse_kinematics_solver::frame2Link(gsGoPath[i].transition[j], locoPlanner.variables);
          for(int b=0; b<param->bodies.size(); b++) {
            param->bodies[b]->calcForwardKinematics(false);
            param->bodies[b]->calcCenterOfMass();
          }
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / gsGoPath[i].transition.size()));
        }
        global_inverse_kinematics_solver::frame2Link(gsGoPath[i].frame, locoPlanner.variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }

      for(int i=0;i<gsDetachPath.size();i++){
        for (int j=0;j<gsDetachPath[i].transition.size();j++) {
          global_inverse_kinematics_solver::frame2Link(gsDetachPath[i].transition[j], manipPlanner.variables);
          for(int b=0; b<param->bodies.size(); b++) {
            param->bodies[b]->calcForwardKinematics(false);
            param->bodies[b]->calcCenterOfMass();
          }
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / gsDetachPath[i].transition.size()));
        }
        global_inverse_kinematics_solver::frame2Link(gsDetachPath[i].frame, manipPlanner.variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }

    }
  }
}
