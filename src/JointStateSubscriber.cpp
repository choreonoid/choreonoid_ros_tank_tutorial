#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/LazyCaller>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace cnoid;

class JointStateSubscriber : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    BodyItemPtr bodyItem;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        bodyItem = static_cast<BodyItem*>(config->bodyItem());
        node.reset(new ros::NodeHandle(bodyItem->name()));
        subscriber = node->subscribe(
            string("/") + bodyItem->name() + "/joint_state",
            1,
            &JointStateSubscriber::jointStateCallback, this);
        return true;
    }

    void jointStateCallback(const sensor_msgs::JointState& state)
    {
        callLater([this, state](){ updateJointState(state); });
    }

    void updateJointState(const sensor_msgs::JointState& state)
    {
        auto body = bodyItem->body();
        auto& names = state.name;
        auto& positions = state.position;
        int size = std::min(names.size(), positions.size());
        int n = std::min(body->numJoints(), size);
        for(int i=0; i < n; ++i){
            auto joint = body->joint(i);
            if(joint->jointName() == names[i]){
                joint->q() = positions[i];
            }
        }
        bodyItem->notifyKinematicStateChange(true);
    }

    virtual void unconfigure() override
    {
        bodyItem.reset();
        node.reset();
        subscriber = ros::Subscriber();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointStateSubscriber)
