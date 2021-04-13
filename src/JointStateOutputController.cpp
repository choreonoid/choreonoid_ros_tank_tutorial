#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace cnoid;

class JointStateOutputController : public SimpleController
{
    BodyPtr ioBody;
    std::unique_ptr<ros::NodeHandle> node;
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        node.reset(new ros::NodeHandle(config->body()->name()));
        jointStatePublisher = node->advertise<sensor_msgs::JointState>("joint_state", 1);
        return true;
    }
        
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        int n = ioBody->numJoints();
        jointState.name.resize(n);
        jointState.position.resize(n);
        jointState.velocity.resize(n);
        jointState.effort.resize(n);

        for(int i=0; i < n; ++i) {
            auto joint = ioBody->joint(i);
            io->enableInput(joint, JointDisplacement | JointVelocity | JointEffort);
            jointState.name[i] = joint->name();
        }

        time = 0.0;
        timeStep = io->timeStep();
        const double frequency = 50.0;
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;

        return true;
    }

    virtual bool control() override
    {
        time += timeStep;
        timeCounter += timeStep;

        if(timeCounter >= cycleTime) {
            
            jointState.header.stamp.fromSec(time);

            for(int i=0; i < ioBody->numJoints(); ++i) {
                auto joint = ioBody->joint(i);
                jointState.position[i] = joint->q();
                jointState.velocity[i] = joint->dq();
                jointState.effort[i] = joint->u();
            }
            
            jointStatePublisher.publish(jointState);

            timeCounter -= cycleTime;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointStateOutputController)
