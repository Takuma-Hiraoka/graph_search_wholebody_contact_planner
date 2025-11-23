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
    // searchRegionConstraints
    {
      // pitch < 90
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = robot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      param->searchRegionConstraints.push_back(constraint);
    }
    {
      // pitch > 0
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.1,0.0,0.0);
      constraint->B_link() = robot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      param->searchRegionConstraints.push_back(constraint);
    }

    // goal
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->B_localpos() = cube->rootLink()->T();
      constraint->weight() << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
      constraint->precision() = 0.7;
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
    //    keyPosePath = cwcpPath;

    global_inverse_kinematics_solver::frame2Link(initialPose, param->variables);

    graph_search_wholebody_contact_planner::WholeBodyLocomotionContactPlanner locoPlanner;
    graph_search_wholebody_contact_planner::convertCWCPParam(*param, keyPosePath, locoPlanner);
    global_inverse_kinematics_solver::link2Frame(locoPlanner.variables, locoPlanner.currentContactState->frame);
    // planner.rejections
    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> csc_;
    choreonoid_contact_candidate_generator::generateCC(obstacle, csc_, 0.1);
    for (int i=0; i<csc_.size(); i++) {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> cc = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      cc->bodyName = csc_[i].body_name;
      cc->linkName = csc_[i].link_name;
      cc->localPose.translation() = csc_[i].p.cast<double>();
      cc->localPose.linear() = csc_[i].R.cast<double>();
      cc->isStatic = true;
      locoPlanner.contactStaticCandidates.push_back(cc);
    }

    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> cdc_;
    choreonoid_contact_candidate_generator::generateCC(cube, cdc_, 0.1);
    for (int i=0; i<cdc_.size(); i++) {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> cc = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      cc->bodyName = cdc_[i].body_name;
      cc->linkName = cdc_[i].link_name;
      cc->localPose.translation() = cdc_[i].p.cast<double>();
      cc->localPose.linear() = cdc_[i].R.cast<double>();
      cc->isStatic = true;
      locoPlanner.contactStaticCandidates.push_back(cc);
    }
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> rleg = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      rleg->bodyName = robot->name();
      rleg->linkName = "RLEG_JOINT5";
      rleg->isStatic = false;
      rleg->localPose.translation() = cnoid::Vector3(0,0,-0.1);
      locoPlanner.contactDynamicCandidates.push_back(rleg);
    }
    // lleg
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> lleg = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      lleg->bodyName = robot->name();
      lleg->linkName = "LLEG_JOINT5";
      lleg->isStatic = false;
      lleg->localPose.translation() = cnoid::Vector3(0,0,-0.1);
      locoPlanner.contactDynamicCandidates.push_back(lleg);
    }
    // rarm
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> rarm = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      rarm->bodyName = robot->name();
      rarm->linkName = "RARM_JOINT7";
      rarm->isStatic = false;
      rarm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
      locoPlanner.contactDynamicCandidates.push_back(rarm);
    }
    // larm
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> larm = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      larm->bodyName = robot->name();
      larm->linkName = "LARM_JOINT7";
      larm->isStatic = false;
      larm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
      locoPlanner.contactDynamicCandidates.push_back(larm);
    }
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    // std::vector<cnoid::SgNodePtr> csc = graph_search_wholebody_contact_planner::generateCandidateMakers(locoPlanner.bodies, locoPlanner.contactStaticCandidates);
    // std::vector<cnoid::SgNodePtr> cdc = graph_search_wholebody_contact_planner::generateCandidateMakers(locoPlanner.bodies, locoPlanner.contactDynamicCandidates);
    // drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    // drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > larmGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "LARM_JOINT7", larmGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > rarmGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "RARM_JOINT7", rarmGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > llegGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "LLEG_JOINT5", llegGuidedCandidates);
    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > rlegGuidedCandidates;
    locoPlanner.candidatesFromGuide(locoPlanner.bodies, locoPlanner.contactStaticCandidates, locoPlanner.guidePath, "JAXON", "RLEG_JOINT5", rlegGuidedCandidates);
    cnoid::BodyPtr ccMarkers = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      ccMarkers->setRootLink(rootLink);
      graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, larmGuidedCandidates, cnoid::Vector3f(1.0, 0.0, 0.0));
      graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, rarmGuidedCandidates, cnoid::Vector3f(0.0, 0.0, 1.0));
      graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, llegGuidedCandidates, cnoid::Vector3f(0.5, 0.5, 0.0));
      graph_search_wholebody_contact_planner::generateCandidateVisualLink(locoPlanner.bodies, rootLink, rlegGuidedCandidates, cnoid::Vector3f(0.0, 0.5, 0.5));
    }
    viewer->objects(ccMarkers);

    viewer->drawOn(drawOnObjects);
    viewer->drawObjects();

    // locoPlanner.pikParam.debugLevel = 3;
    // locoPlanner.pikParam.viewMilliseconds = -1;
    // locoPlanner.pikParam.viewer = viewer;
    locoPlanner.viewer = viewer;
    locoPlanner.bodyContactConstraints.push_back(generateBodyContactConstraint(locoPlanner.bodies, robot->link("LARM_JOINT7"), 0.02));
    locoPlanner.bodyContactConstraints.push_back(generateBodyContactConstraint(locoPlanner.bodies, robot->link("RARM_JOINT7"), 0.02));
    locoPlanner.bodyContactConstraints.push_back(generateBodyContactConstraint(locoPlanner.bodies, robot->link("LLEG_JOINT5"), 0.02));
    locoPlanner.bodyContactConstraints.push_back(generateBodyContactConstraint(locoPlanner.bodies, robot->link("RLEG_JOINT5"), 0.02));

    locoPlanner.addCandidateDistance = 1.5;
    locoPlanner.addNearGuideCandidateDistance = 0.2;
    locoPlanner.currentContactState->transition.push_back(locoPlanner.currentContactState->frame);
    locoPlanner.threads() = 20;
    locoPlanner.debugLevel() = 0;
    // locoPlanner.maxExtendNum() = 1000;

    locoPlanner.solve();

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsGoPath;
    locoPlanner.goalPath(gsGoPath);

    graph_search_wholebody_contact_planner::WholeBodyManipulationContactPlanner ManipPlanner;

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

    }
  }
}
