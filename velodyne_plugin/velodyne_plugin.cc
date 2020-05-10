#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo
{
    class VelodynePlugin : public ModelPlugin{
        public: VelodynePlugin() {}

            private: physics::ModelPtr model;
            private: physics::JointPtr joint;
            private: common::PID pid;
            public: virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf){
                if (_model->GetJointCount() == 0){
                    std::cerr << "Invalid joint count, Velodyne plugin notloaded\n";
                    return;
                }
            this->model = _model;
            this->pid = common::PID(0.1, 0, 0);
            this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), 10.0);
            double velocity = 0;
            if (_sdf->HasElement("velocity"))
                velocity = _sdf->Get<double>("velocity");
            this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), velocity);
        }

    };
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin);
}
#endif
