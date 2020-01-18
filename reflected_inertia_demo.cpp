/*
 * Demo Example to investigate mimic joint behavior.  
 * Creates a two pendulum system. The 1st link is a simple traditional revolute joint.
 * The second link is modeling an unpowered gearmotor; while also a revolute joint, there
 * is a second body representing the motor's rotor which due to the gear ratio is
 * constrained to rotate much faster than the output link. 
 *
 */

#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

const double link_length = 1.0; 
const double link_width = 0.1;
const double link_mass = 4;

const double default_damping = 0.1;

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::glut::SimWindow{
public:
    MyWindow(WorldPtr world){
        setWorld(world);
    }


    void timeStepping() override {
        SimWindow::timeStepping();
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setLinkGeometry(const BodyNodePtr& bn){
    // Create a BoxShape to be used for both visualization and collision checking
    std::shared_ptr<BoxShape> box(new BoxShape(
        Eigen::Vector3d(link_width, link_width, link_length)));

    // Create a shape node for visualization and collision checking
    auto shapeNode  
      = bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(box);
          shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    
    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, link_length / 2.0);
    box_tf.translation() = center;
    shapeNode->setRelativeTransform(box_tf);

    // Move the center of mass to the center of the object
    bn->setLocalCOM(center);
}

BodyNode* makeRootLink(const SkeletonPtr& pendulum, const std::string& name){
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mRestPositions[0] = 0.0;
    properties.mSpringStiffnesses[0] = 0.0;
    properties.mDampingCoefficients[0] = default_damping;

    BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, properties, BodyNode::AspectProperties(name)).second;

    // Make a shape for the Joint
    const double R = link_width;
    const double h = link_width * 2.0;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);

    // Set the geometry of the Body
    setLinkGeometry(bn);

    std::cout << "added link of mass " << bn->getMass() << " kg" << std::endl;
    
    return bn;
} 

BodyNode* addLink(const SkeletonPtr& pendulum, BodyNode* parent,
                  const std::string& name, Eigen::Vector3d translation){
    // Set up the properties for the Joint
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ParentBodyToJoint.translation() = translation;
    properties.mRestPositions[0] = 0.0;
    properties.mSpringStiffnesses[0] = 0.0;
    properties.mDampingCoefficients[0] = default_damping;

    // Create a new BodyNode, attached to its parent by a RevoluteJoint
    BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

    // Make a shape for the Joint
    const double R = link_width / 2.0;
    const double h = link_width * 2.0;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);

    // Set the geometry of the Body
    setLinkGeometry(bn);

    std::cout << "added link of mass " << bn->getMass() << " kg" << std::endl;

    return bn;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this is the key function
// "rotor" here represents the simulated motor rotor of an actuator consisting of an electric motor and a gearbox of ratio
// the effects of reflected inertia due to the rotor significanty impact system dynamics so it is important to model
// rotor is 
BodyNode* addRotor(const SkeletonPtr& pendulum, BodyNode* parent, BodyNode* output,
		const std::string& name, Eigen::Vector3d translation){
    // Set up the properties for the Joint
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    //properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0.0, link_width, 0.0);
    properties.mT_ParentBodyToJoint = output->getTransform(parent, output);
    properties.mRestPositions[0] = 0.0;
    properties.mSpringStiffnesses[0] = 0.0;
    properties.mDampingCoefficients[0] = default_damping;

    // Create a new BodyNode, attached to its parent by a RevoluteJoint
    BodyNodePtr rotor = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

    rotor->setMass(0.1);

    // Make a shape for the Joint
    const double R = 4.0* link_width / 2.0;
    const double h = link_width;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

    auto shapeNode = rotor->createShapeNodeWith<VisualAspect, DynamicsAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    shapeNode->setRelativeTransform(tf);
    rotor->setLocalCOM(Eigen::Vector3d(0, 0, 0));

    const double ratio = 2.0;
    rotor->getParentJoint()->setActuatorType(Joint::MIMIC);
    rotor->getParentJoint()->setMimicJoint(output->getParentJoint(), ratio, 0.0);

    std::cout << "added rotor of mass " << rotor->getMass() << " kg" << std::endl;

    return rotor;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]){
  std::cout << "starting main!" << std::endl;
  WorldPtr world = World::create();

  // Create an empty Skeleton with the name "pendulum"
  SkeletonPtr pendulum = Skeleton::create("pendulum");

  // Add each body to the last BodyNode in the pendulum
  BodyNode* link1 = makeRootLink(pendulum, "link1");
  BodyNode* link2 = addLink(pendulum, link1, "link2", Eigen::Vector3d(0, 0, link_length));
                    addRotor(pendulum, link1, link2, "rotor", Eigen::Vector3d(0.0, 0.0, 0.0));

  // Set the initial position of the first DegreeOfFreedom so that the pendulum starts to swing right away
  pendulum->getDof(0)->setPosition(160.0 * M_PI / 180.0);

  // Create a world and add the pendulum to the world
  world->addSkeleton(pendulum);

  // Change the solver type to try to do better for the gearbox?
  // auto lcpSolver = std::make_shared<dart::constraint::PgsBoxedLcpSolver>();
  // auto solver = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>(lcpSolver);
  // world->setConstraintSolver(std::move(solver));

  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(1280, 960, "Multi-Pendulum Tutorial");
  std::cout << "all setup, here goes..." << std::endl;
  glutMainLoop();
}

