#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/LazyCaller>
#include <cnoid/Timer>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <random>

using namespace std;
using namespace cnoid;

class RttImuStateSubscriber : public SimpleController
{
    unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    BodyItemPtr bodyItem;
    AccelerationSensorPtr accelSensor;
    RateGyroSensorPtr gyro;
    Timer bodyShakeTimer;
    double bodyShakeDuration;
    Isometry3 bodyPosition;
    mt19937 mt;
    uniform_real_distribution<> rand;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        bodyItem = static_cast<BodyItem*>(config->bodyItem());

        auto body = bodyItem->body();

        accelSensor = body->findDevice<AccelerationSensor>();
        if(accelSensor){
            auto sigTimeout = bodyShakeTimer.sigTimeout();
            if(!sigTimeout.hasConnections()){
                sigTimeout.connect([this](){ onBodyShakeTimerTimeout(); });
                bodyShakeTimer.setInterval(20);
            }
        }
        
        gyro = body->findDevice<RateGyroSensor>();
        
        node.reset(new ros::NodeHandle(bodyItem->name()));
        subscriber = node->subscribe(
            string("/") + bodyItem->name() + "/imu",
            1,
            &RttImuStateSubscriber::imuStateCallback, this);
        
        return true;
    }

    void imuStateCallback(const sensor_msgs::Imu& state)
    {
        callLater([this, state](){ updateImuState(state); });
    }

    void updateImuState(const sensor_msgs::Imu& state)
    {
        if(accelSensor){
            auto& dv = state.linear_acceleration;
            accelSensor->dv() << dv.x, dv.y, dv.z;
            accelSensor->notifyStateChange();
            if(accelSensor->dv().head<2>().norm() > 20.0){
                startBodyShake();
            }
        }
        if(gyro){
            auto& w = state.angular_velocity;
            gyro->w() << w.x, w.y, w.z;
            gyro->notifyStateChange();
        }
    }

    void startBodyShake()
    {
        bodyShakeDuration = 0.5;
        if(!bodyShakeTimer.isActive()){
            bodyPosition = bodyItem->body()->rootLink()->position();
            rand.param(uniform_real_distribution<>::param_type(-0.02, 0.02));
            bodyShakeTimer.start();
        }
    }

    void onBodyShakeTimerTimeout()
    {
        if(bodyShakeDuration > 0.0){
            auto T = bodyPosition;
            T.translation() += Vector3(rand(mt), rand(mt), rand(mt));
            bodyItem->body()->rootLink()->setPosition(T);
        } else {
            bodyShakeTimer.stop();
            bodyItem->body()->rootLink()->setPosition(bodyPosition);
        }
        bodyItem->notifyKinematicStateChange();
        bodyShakeDuration -= 0.02;
    }

    virtual void unconfigure() override
    {
        node.reset();
        subscriber = ros::Subscriber();
        bodyItem.reset();
        accelSensor.reset();
        gyro.reset();
        bodyShakeTimer.stop();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttImuStateSubscriber)
