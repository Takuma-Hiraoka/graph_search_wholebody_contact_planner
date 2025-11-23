#include <choreonoid_viewer/choreonoid_viewer.h>

#include <graph_search_wholebody_contact_planner/locomotion_planner.h>
#include <choreonoid_contact_candidate_generator/choreonoid_contact_candidate_generator.h>
#include "jaxon_common.h"
#include "world_common.h"

namespace graph_search_wholebody_contact_planner_sample{
  void sample0_locomotion(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<cwcp::CWCPParam> param = std::make_shared<cwcp::CWCPParam>();
    generateWallWorld(obstacle, param);

    cnoid::BodyPtr robot;
    generateJAXON(robot, param);

    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);
    // goal
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->B_localpos() = robot->rootLink()->T();
      constraint->B_localpos().translation() += cnoid::Vector3(0.5,0,0.5);
      constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      constraint->precision() = 1e-3;
      param->goals.push_back(constraint);
    }

    param->gikParam.goalBias = 0.4;
    param->OptimizeTrajectory = true; // 軌道最適化を行うと外れ値的な接触や接触の数自体を減らせるが、特にはじめに一気に飛んでしまう
    param->toParam.initialShortcut = true;
    param->toParam.shortcut = true;
    param->toParam.shortcutThre = param->gikParam.delta/2; // 小さくしすぎるとIKが解けない確率が増える

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    for(int b=0; b<param->bodies.size(); b++) viewer->objects(param->bodies[b]);
    viewer->drawObjects();

    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > > cwcpPath;
    if(!cwcp::solveCWCP(param, cwcpPath)) std::cerr << "solveCWCP failed" << std::endl;
    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > > keyPosePath;
    if(!cwcp::generateKeyPose(param, cwcpPath, keyPosePath)) std::cerr << "generateKeyPose failed" << std::endl;

    global_inverse_kinematics_solver::frame2Link(initialPose, param->variables);

    graph_search_wholebody_contact_planner::WholeBodyLocomotionContactPlanner planner;
    graph_search_wholebody_contact_planner::convertCWCPParam(*param, keyPosePath, planner);
    global_inverse_kinematics_solver::link2Frame(planner.variables, planner.currentContactState->frame);
    // planner.rejections
    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> csc_;
    choreonoid_contact_candidate_generator::generateCC(obstacle, csc_, 0.1);
    graph_search_wholebody_contact_planner::convertContactCandidates(csc_, planner.contactStaticCandidates, true);

    addLimbInfo(planner, robot);

    std::vector<cnoid::SgNodePtr> drawOnObjects;
    std::vector<cnoid::SgNodePtr> csc = graph_search_wholebody_contact_planner::generateCandidateMarkers(planner.bodies, planner.contactStaticCandidates);
    std::vector<cnoid::SgNodePtr> cdc = graph_search_wholebody_contact_planner::generateCandidateMarkers(planner.bodies, planner.contactDynamicCandidates);
    // drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    // drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    std::vector<std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> > guidedCandidates;
    planner.candidatesFromGuide(planner.bodies, planner.contactStaticCandidates, planner.guidePath, "JAXON", "LARM_JOINT7", guidedCandidates);
    planner.candidatesFromGuide(planner.bodies, planner.contactStaticCandidates, planner.guidePath, "JAXON", "RARM_JOINT7", guidedCandidates);
    planner.candidatesFromGuide(planner.bodies, planner.contactStaticCandidates, planner.guidePath, "JAXON", "LLEG_JOINT5", guidedCandidates);
    planner.candidatesFromGuide(planner.bodies, planner.contactStaticCandidates, planner.guidePath, "JAXON", "RLEG_JOINT5", guidedCandidates);

    cnoid::BodyPtr ccMarkers = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      ccMarkers->setRootLink(rootLink);
      graph_search_wholebody_contact_planner::generateCandidateVisualLink(planner.bodies, rootLink, guidedCandidates, cnoid::Vector3f(1.0, 0.0, 0.0));
    }
    drawOnObjects.push_back(ccMarkers->rootLink()->visualShape());
    viewer->drawOn(drawOnObjects);
    viewer->drawObjects();


    planner.pikParam.debugLevel = 0;
    planner.pikParam.viewMilliseconds = -1;
    // planner.pikParam.viewer = viewer;

    planner.addCandidateDistance = 1.2;
    planner.addNearGuideCandidateDistance = 0.1;
    planner.threads() = 10;
    planner.debugLevel() = 0;
    // planner.maxExtendNum() = 10;
    planner.solve();

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsPath;
    planner.goalPath(gsPath);

    while (true) {
      for(int i=0;i<cwcpPath.size();i++){
        global_inverse_kinematics_solver::frame2Link(cwcpPath.at(i).first,param->variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        std::vector<cnoid::SgNodePtr> markers;
        for (int j=0;j<cwcpPath.at(i).second.size();j++) {
          cnoid::SgLineSetPtr lines_ = new cnoid::SgLineSet;
          lines_->setLineWidth(8.0);
          lines_->getOrCreateColors()->resize(1);
          lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.9,0.9,0.0);
          lines_->getOrCreateVertices()->resize(2);
          lines_->colorIndices().resize(0);
          lines_->addLine(0,1); lines_->colorIndices().push_back(0); lines_->colorIndices().push_back(0);
          const std::vector<cnoid::SgNodePtr>& marker = std::vector<cnoid::SgNodePtr>{lines_};
          lines_->getOrCreateVertices()->at(0) = (cwcpPath.at(i).second[j]->c1.link->T() * cwcpPath.at(i).second[j]->c1.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
          lines_->getOrCreateVertices()->at(1) = (cwcpPath.at(i).second[j]->c2.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        viewer->drawOn(markers);
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      for(int i=0;i<keyPosePath.size();i++){
        global_inverse_kinematics_solver::frame2Link(keyPosePath.at(i).first,param->variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        std::vector<cnoid::SgNodePtr> markers;
        for (int j=0;j<keyPosePath.at(i).second.size();j++) {
          cnoid::SgLineSetPtr lines_ = new cnoid::SgLineSet;
          lines_->setLineWidth(8.0);
          lines_->getOrCreateColors()->resize(1);
          lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.9,0.9,0.0);
          lines_->getOrCreateVertices()->resize(2);
          lines_->colorIndices().resize(0);
          lines_->addLine(0,1); lines_->colorIndices().push_back(0); lines_->colorIndices().push_back(0);
          const std::vector<cnoid::SgNodePtr>& marker = std::vector<cnoid::SgNodePtr>{lines_};
          lines_->getOrCreateVertices()->at(0) = (keyPosePath.at(i).second[j]->c1.link->T() * keyPosePath.at(i).second[j]->c1.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
          lines_->getOrCreateVertices()->at(1) = (keyPosePath.at(i).second[j]->c2.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        viewer->drawOn(markers);
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      for(int i=0;i<gsPath.size();i++){
        for (int j=0;j<gsPath[i].transition.size();j++) {
          global_inverse_kinematics_solver::frame2Link(gsPath[i].transition[j], planner.variables);
          for(int b=0; b<param->bodies.size(); b++) {
            param->bodies[b]->calcForwardKinematics(false);
            param->bodies[b]->calcCenterOfMass();
          }
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / gsPath[i].transition.size()));
        }
        global_inverse_kinematics_solver::frame2Link(gsPath[i].frame, planner.variables);
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
