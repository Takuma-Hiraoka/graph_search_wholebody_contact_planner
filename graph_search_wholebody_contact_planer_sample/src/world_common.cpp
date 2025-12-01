#include "world_common.h"

#include <cnoid/MeshGenerator>
#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace graph_search_wholebody_contact_planner_sample{
  void generateWallWorld(cnoid::BodyPtr& obstacle,
                         const std::shared_ptr<cwcp::CWCPParam>& param
                         ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        rootLink->setName("floor");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2,2,0.1)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,-0.05);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      cnoid::LinkPtr lWall = new cnoid::Link();
      {
        lWall->setName("lWall");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,0.2,1.75)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,0.65,0.9);
        posTransform->addChild(shape);
        lWall->addShapeNode(posTransform);
        rootLink->appendChild(lWall);
      }
      cnoid::LinkPtr rWall = new cnoid::Link();
      {
        rWall->setName("rWall");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,0.2,1.75)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,-0.65,0.9);
        posTransform->addChild(shape);
        rWall->addShapeNode(posTransform);
        rootLink->appendChild(rWall);
      }
      obstacle->setRootLink(rootLink);
    }
    {
      // collision world
      param->field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                               5,//size_y
                                                                                               5,//size_z
                                                                                               0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                               -2.5,//origin_x
                                                                                               -2.5,//origin_y
                                                                                               -0.5,//origin_z
                                                                                               0.5, // max_distance
                                                                                               true// propagate_negative_distances
                                                                                               );
      EigenSTL::vector_Vector3d vertices;
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->rootLink(), 0.01);
      for(int j=0;j<vertices_.size();j++){
        vertices.push_back(obstacle->rootLink()->T() * vertices_[j].cast<double>());
      }
      param->field->addPointsToField(vertices);
    }
    param->bodies.push_back(obstacle);
  }

  void generateTableWorld(cnoid::BodyPtr& obstacle,
                          const std::shared_ptr<cwcp::CWCPParam>& param) {
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    obstacle->setName("floor");
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        rootLink->setName("floor");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2.2,2.2,0.1)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,-0.05);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      {
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,0,0.45);
        posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      {
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0.5,0.45);
        posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      cnoid::LinkPtr table1 = new cnoid::Link();
      {
        table1->setName("table1");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.5,0.5,0.1)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,0,1.0-0.05);
        posTransform->addChild(shape);
        table1->addShapeNode(posTransform);
        rootLink->appendChild(table1);
      }
      cnoid::LinkPtr table2 = new cnoid::Link();
      {
        table2->setName("table2");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.5,0.5,0.1)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0.5,1.0-0.05);
        posTransform->addChild(shape);
        table2->addShapeNode(posTransform);
        rootLink->appendChild(table2);
      }
      obstacle->setRootLink(rootLink);
    }
    param->field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                             5,//size_y
                                                                                             5,//size_z
                                                                                             0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                             -2.5,//origin_x
                                                                                             -2.5,//origin_y
                                                                                             -2.5,//origin_z
                                                                                             0.5, // max_distance
                                                                                             true// propagate_negative_distances
                                                                                             );
    EigenSTL::vector_Vector3d vertices;
    for(int i=0;i<obstacle->numLinks();i++){
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
      for(int j=0;j<vertices_.size();j++){
        vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
      }
    }
    param->field->addPointsToField(vertices);
    param->bodies.push_back(obstacle);
  }
  void generateHallWorld(cnoid::BodyPtr& obstacle,
                         const std::shared_ptr<cwcp::CWCPParam>& param) {
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    obstacle->setName("floor");
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      rootLink->setName("floor");
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(5,2,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(5,2,0.1)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.9);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(1.0,0,2.05);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(5,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.9);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(1.0,0.95,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(5,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.9);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(1.0,-0.95,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.6,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(1.0,0.35,1.0);
        //   posTransform->translation() = cnoid::Vector3(1.0,0.45,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.6,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(1.0,-0.35,1.0);
        //   posTransform->translation() = cnoid::Vector3(1.0,-0.45,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(0.3,-0.6,1.0);
        //   posTransform->translation() = cnoid::Vector3(0.3,-0.7,1.0);
        //   posTransform->rotation() = cnoid::rotFromRpy(0,0,M_PI / 6);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(0.3,0.6,1.0);
        //   posTransform->translation() = cnoid::Vector3(0.3,0.7,1.0);
        //   posTransform->rotation() = cnoid::rotFromRpy(0,0,-M_PI / 6);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(1.7,-0.6,1.0);
        //   posTransform->translation() = cnoid::Vector3(1.7,-0.7,1.0);
        //   posTransform->rotation() = cnoid::rotFromRpy(0,0,-M_PI / 6);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,0.1,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.5);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   // posTransform->translation() = cnoid::Vector3(1.7,0.6,1.0);
        //   posTransform->translation() = cnoid::Vector3(1.7,0.7,1.0);
        //   posTransform->rotation() = cnoid::rotFromRpy(0,0,M_PI / 6);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0);
        //   material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(-0.7,0,0.45);
        //   posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0.7,0,0.45);
          posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.1,1.8,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.9);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(-1.05,0.0,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        // {
        //   cnoid::SgShapePtr shape = new cnoid::SgShape();
        //   shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.1,1.8,2.0)));
        //   cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        //   material->setTransparency(0.9);
        //   shape->setMaterial(material);
        //   cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        //   posTransform->translation() = cnoid::Vector3(3.05,0.0,1.0);
        //   posTransform->addChild(shape);
        //   rootLink->addShapeNode(posTransform);
        // }
        cnoid::LinkPtr table1 = new cnoid::Link();
        {
          table1->setName("table1");
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateCylinder(0.25, 0.1));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.3, 0.2));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0.7,1.0-0.05);
          // posTransform->translation() = cnoid::Vector3(-0.7,0,1.0-0.05);
          posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
          posTransform->addChild(shape);
          table1->addShapeNode(posTransform);
          rootLink->appendChild(table1);
        }
        cnoid::LinkPtr table2 = new cnoid::Link();
        {
          table2->setName("table2");
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateCylinder(0.25, 0.1));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.3, 0.2));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0.7,0,1.0-0.05);
          posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
          posTransform->addChild(shape);
          table2->addShapeNode(posTransform);
          rootLink->appendChild(table2);
        }
      }
      obstacle->setRootLink(rootLink);
      {
        // collision world
        param->field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                 5,//size_y
                                                                                                 5,//size_z
                                                                                                 0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                                 -2.5,//origin_x
                                                                                                 -2.5,//origin_y
                                                                                                 -0.5,//origin_z
                                                                                                 0.5, // max_distance
                                                                                                 true// propagate_negative_distances
                                                                                                 );
        EigenSTL::vector_Vector3d vertices;
        for(int i=0;i<obstacle->numLinks();i++){
          std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
          for(int j=0;j<vertices_.size();j++){
            vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
          }
        }
        param->field->addPointsToField(vertices);
      }
      param->bodies.push_back(obstacle);
    }
  }
  void generateBOX(cnoid::BodyPtr& box,
                   const std::shared_ptr<cwcp::CWCPParam>& param) {
    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyLoader bodyLoader;
    box = bodyLoader.load(ros::package::getPath("eusurdfwrl") + "/models/maxvalu-supermarket-basket/maxvalu-supermarket-basket.wrl");
    param->bodies.push_back(box);
    param->variables.push_back(box->rootLink());
    box->setName("box");
    box->rootLink()->p() = cnoid::Vector3(0.5,0,1.0);
    box->rootLink()->R() = cnoid::rotFromRpy(0.0, 0.0, M_PI/2);
    box->calcForwardKinematics();
    box->calcCenterOfMass();

    // collision
    {
      {
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = box->rootLink();
        constraint->field() = param->field;
        constraint->tolerance() = 0.015; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.010; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする.
        constraint->ignoreDistance() = 0.1; // 大きく動くので、ignoreも大きくする必要がある
        //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        param->constraints.push_back(constraint);
      }
    }
  }
  void generateCUBE(cnoid::BodyPtr& cube,
                    const std::shared_ptr<cwcp::CWCPParam>& param) {
    cnoid::MeshGenerator meshGenerator;
    cube = new cnoid::Body();
    cube->setName("cube");
    cnoid::LinkPtr rootLink = new cnoid::Link();
    {
      rootLink->setName("cube");
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.4,0.4,0.4)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setDiffuseColor(cnoid::Vector3f(0.1, 0.9, 0.1));
      material->setTransparency(0);
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->addChild(shape);
      rootLink->addShapeNode(posTransform);
    }
    cube->setRootLink(rootLink);
    cube->rootLink()->setJointType(cnoid::Link::JointType::FreeJoint);
    param->bodies.push_back(cube);
    // param->variables.push_back(cube->rootLink());
    cube->rootLink()->p() = cnoid::Vector3(0.7,0,1.2);
    cube->calcForwardKinematics();
    cube->calcCenterOfMass();
  }
}
