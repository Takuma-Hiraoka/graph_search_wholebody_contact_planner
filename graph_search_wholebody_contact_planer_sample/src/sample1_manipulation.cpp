#include <choreonoid_viewer/choreonoid_viewer.h>

#include <graph_search_wholebody_contact_planner/manipulation_planner.h>
#include <choreonoid_contact_candidate_generator/choreonoid_contact_candidate_generator.h>
#include "jaxon_common.h"
#include "world_common.h"

namespace graph_search_wholebody_contact_planner_sample{
  void sample1_manipulation(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<cwcp::CWCPParam> param = std::make_shared<cwcp::CWCPParam>();
    generateTableWorld(obstacle, param);

    cnoid::BodyPtr box;
    generateBOX(box, param);

    cnoid::BodyPtr robot;
    generateJAXON(robot, param);

    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    for(int b=0; b<param->bodies.size(); b++) viewer->objects(param->bodies[b]);
    viewer->drawObjects();

    graph_search_wholebody_contact_planner::WholeBodyManipulationContactPlanner planner;
    planner.bodies = param->bodies;
    planner.variables = param->variables;
    planner.constraints = param->constraints;
    planner.nominals = param->nominals;
    planner.currentContactState = std::make_shared<graph_search_wholebody_contact_planner::ContactState>();
    for (int i=0; i<param->currentContactPoints.size(); i++) {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      for (int b=0; b<param->bodies.size(); b++) {
        if (param->bodies[b]->link(param->currentContactPoints[i]->c1.link->name())) {
          c1.bodyName = param->bodies[b]->name();
          c1.linkName = param->currentContactPoints[i]->c1.link->name();
          c1.localPose = param->currentContactPoints[i]->c1.localPose;
          c1.isStatic = false;
        }
      }
      {
        c2.localPose = param->currentContactPoints[i]->c2.localPose;
        c2.localPose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
        c2.isStatic = true;
      }
      planner.currentContactState->contacts.push_back(graph_search_wholebody_contact_planner::Contact(c1,c2));
    }
    {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      c1.bodyName = box->name();
      c1.linkName = "BOX_link";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, 0.0);
      c1.localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, -M_PI/2);
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      c2.bodyName = "floor";
      c2.linkName = "table1";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(0.5, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      planner.currentContactState->contacts.push_back(graph_search_wholebody_contact_planner::Contact(c1,c2));
    }

    planner.field = param->field;

    global_inverse_kinematics_solver::link2Frame(planner.variables, planner.currentContactState->frame);

    // Goal
    {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      c1.bodyName = box->name();
      c1.linkName = "BOX_link";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, 0.0);
      c1.localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, -M_PI/2);
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      c2.bodyName = "floor";
      c2.linkName = "table1";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(0.5, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      planner.targetContact = std::make_pair(graph_search_wholebody_contact_planner::Contact(c1,c2), false);
    }

    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> csc_;
    choreonoid_contact_candidate_generator::generateCC(obstacle, csc_, 0.2);
    for (int i=0; i<csc_.size(); i++) {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> cc = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      cc->bodyName = csc_[i].body_name;
      cc->linkName = csc_[i].link_name;
      cc->localPose.translation() = csc_[i].p.cast<double>();
      cc->localPose.linear() = csc_[i].R.cast<double>();
      cc->isStatic = true;
      planner.contactStaticCandidates.push_back(cc);
    }

    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> cdc_;
    choreonoid_contact_candidate_generator::generateCC(box->rootLink(), cdc_, 0.1);
    for (int i=0; i<cdc_.size(); i++) {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> cc = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      cc->bodyName = cdc_[i].body_name;
      cc->linkName = cdc_[i].link_name;
      cc->localPose.translation() = cdc_[i].p.cast<double>();
      cc->localPose.linear() = cdc_[i].R.cast<double>();
      cc->isStatic = false;
      planner.contactDynamicCandidates.push_back(cc);
    }

    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> rleg = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      rleg->bodyName = robot->name();
      rleg->linkName = "RLEG_JOINT5";
      rleg->isStatic = false;
      rleg->localPose.translation() = cnoid::Vector3(0,0,-0.1);
      planner.contactDynamicCandidates.push_back(rleg);
    }
    // lleg
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> lleg = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      lleg->bodyName = robot->name();
      lleg->linkName = "LLEG_JOINT5";
      lleg->isStatic = false;
      lleg->localPose.translation() = cnoid::Vector3(0,0,-0.1);
      planner.contactDynamicCandidates.push_back(lleg);
    }
    // rarm
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> rarm = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      rarm->bodyName = robot->name();
      rarm->linkName = "RARM_JOINT7";
      rarm->isStatic = false;
      rarm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
      planner.contactDynamicCandidates.push_back(rarm);
    }
    // larm
    {
      std::shared_ptr<graph_search_wholebody_contact_planner::ContactCandidate> larm = std::make_shared<graph_search_wholebody_contact_planner::ContactCandidate>();
      larm->bodyName = robot->name();
      larm->linkName = "LARM_JOINT7";
      larm->isStatic = false;
      larm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
      planner.contactDynamicCandidates.push_back(larm);
    }
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    std::vector<cnoid::SgNodePtr> csc = graph_search_wholebody_contact_planner::generateCandidateMakers(planner.bodies, planner.contactStaticCandidates);
    std::vector<cnoid::SgNodePtr> cdc = graph_search_wholebody_contact_planner::generateCandidateMakers(planner.bodies, planner.contactDynamicCandidates);
    drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    viewer->drawOn(drawOnObjects);
    viewer->drawObjects();


    planner.pikParam.debugLevel = 0;
    planner.pikParam.viewMilliseconds = -1;
    // planner.pikParam.viewer = viewer;
    planner.viewer = viewer;
    planner.addCandidateDistance = 2.0;
    planner.bodyContactConstraints.push_back(generateBodyContactConstraint(planner.bodies, robot->link("LARM_JOINT7"), 0.02));
    planner.bodyContactConstraints.push_back(generateBodyContactConstraint(planner.bodies, robot->link("RARM_JOINT7"), 0.02));
    planner.bodyContactConstraints.push_back(generateBodyContactConstraint(planner.bodies, robot->link("LLEG_JOINT5"), 0.02));
    planner.bodyContactConstraints.push_back(generateBodyContactConstraint(planner.bodies, robot->link("RLEG_JOINT5"), 0.02));

    planner.addCandidateDistance = 1.0;
    planner.currentContactState->transition.push_back(planner.currentContactState->frame);
    planner.threads() = 20;
    planner.debugLevel() = 0;
    planner.maxExtendNum() = 1e8;
    planner.solve();
    std::cerr << "solved" << std::endl;

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsPath;
    planner.goalPath(gsPath);

    while (true) {
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
