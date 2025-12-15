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
    graph_search_wholebody_contact_planner::convertContactCandidates(csc_, planner.contactStaticCandidates, true);

    std::vector<choreonoid_contact_candidate_generator::ContactCandidate> cdc_;
    choreonoid_contact_candidate_generator::generateCC(box->rootLink(), cdc_, 0.1);
    graph_search_wholebody_contact_planner::convertContactCandidates(cdc_, planner.contactDynamicCandidates, false);

    addLimbInfo(planner, robot);

    std::vector<cnoid::SgNodePtr> drawOnObjects;
    std::vector<cnoid::SgNodePtr> csc = graph_search_wholebody_contact_planner::generateCandidateMarkers(planner.bodies, planner.contactStaticCandidates);
    std::vector<cnoid::SgNodePtr> cdc = graph_search_wholebody_contact_planner::generateCandidateMarkers(planner.bodies, planner.contactDynamicCandidates);
    drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    viewer->drawOn(drawOnObjects);
    viewer->drawObjects();


    planner.pikParam.debugLevel = 0;
    planner.pikParam.viewMilliseconds = -1;
    // planner.pikParam.viewer = viewer;

    planner.robotLinkPriority = std::vector<std::vector<std::string> >{{"LEG_JOINT"},{"ARM_JOINT"}};
    planner.addCandidateDistance = 1.0;
    planner.threads() = 10;
    planner.debugLevel() = 0;
    planner.maxExtendNum() = 1e8;
    if (planner.solve()) std::cerr << "detach solved" << std::endl;
    else std::cerr << "detach failed" << std::endl;

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsDetachPath;
    planner.goalPath(gsDetachPath);

    planner.graph().clear();
    planner.setGoal(nullptr);
    global_inverse_kinematics_solver::frame2Link(gsDetachPath[gsDetachPath.size()-1].frame, planner.variables);
    global_inverse_kinematics_solver::link2Frame(planner.variables, planner.currentContactState->frame);
    for(int b=0; b<param->bodies.size(); b++) {
      param->bodies[b]->calcForwardKinematics(false);
      param->bodies[b]->calcCenterOfMass();
    }
    planner.currentContactState->contacts = gsDetachPath[gsDetachPath.size()-1].contacts;
    // Goal
    {
      graph_search_wholebody_contact_planner::ContactCandidate c1;
      c1.bodyName = box->name();
      c1.linkName = "BOX_link";
      c1.isStatic = false;
      graph_search_wholebody_contact_planner::ContactCandidate c2;
      c2.bodyName = "floor";
      c2.linkName = "table2";
      c2.isStatic = true;
      planner.targetContact = std::make_pair(graph_search_wholebody_contact_planner::Contact(c1,c2), true);
    }
    std::cerr << *planner.currentContactState << std::endl;
    if (planner.solve()) std::cerr << "attach solved" << std::endl;
    else std::cerr << "attach failed" << std::endl;

    std::vector<graph_search_wholebody_contact_planner::ContactState> gsAttachPath;
    planner.goalPath(gsAttachPath);

    while (true) {
      for(int i=0;i<gsDetachPath.size();i++){
        for (int j=0;j<gsDetachPath[i].transition.size();j++) {
          global_inverse_kinematics_solver::frame2Link(gsDetachPath[i].transition[j], planner.variables);
          for(int b=0; b<param->bodies.size(); b++) {
            param->bodies[b]->calcForwardKinematics(false);
            param->bodies[b]->calcCenterOfMass();
          }
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / gsDetachPath[i].transition.size()));
        }
        global_inverse_kinematics_solver::frame2Link(gsDetachPath[i].frame, planner.variables);
        for(int b=0; b<param->bodies.size(); b++) {
          param->bodies[b]->calcForwardKinematics(false);
          param->bodies[b]->calcCenterOfMass();
        }
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }

      for(int i=0;i<gsAttachPath.size();i++){
        for (int j=0;j<gsAttachPath[i].transition.size();j++) {
          global_inverse_kinematics_solver::frame2Link(gsAttachPath[i].transition[j], planner.variables);
          for(int b=0; b<param->bodies.size(); b++) {
            param->bodies[b]->calcForwardKinematics(false);
            param->bodies[b]->calcCenterOfMass();
          }
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / gsAttachPath[i].transition.size()));
        }
        global_inverse_kinematics_solver::frame2Link(gsAttachPath[i].frame, planner.variables);
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
