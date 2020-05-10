#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
namespace gazebo
{
class VelodynePlugin : public ModelPlugin{
public: VelodynePlugin() {}

private: physics::ModelPtr model;
private: physics::JointPtr joint;
private: common::PID pid;
    /// \brief Узел, который используется для обмена информацией
private: transport::NodePtr node;
    /// \brief Подписчик
private: transport::SubscriberPtr sub;
    /// \brief Узел
private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief Подписчик
private: ros::Subscriber rosSub;
    /// \brief Очередь входящих сообщений
private: ros::CallbackQueue rosQueue;
    /// \brief Поток, в котором работает очередь входящих сообщений
private: std::thread rosQueueThread;

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

        // Создаём узел
        this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
#else
        this->node->Init(this->model->GetWorld()->Name());
#endif
        // Формируем имя топика
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
        // Подписываемся на топик и указываем функцию обратного вызова
        this->sub = this->node->Subscribe(topicName,
                                          &VelodynePlugin::OnMsg, this);

        // Инициализируем ROS.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }
        // Создаём узел ROS
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
        // Подписываемся на топик
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/vel_cmd",1,boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);this->rosSub = this->rosNode->subscribe(so);
        // Запускаем поток
        this->rosQueueThread =
                std::thread(std::bind(&VelodynePlugin::QueueThread, this));
    }

    /// \brief Обработчик входящих сообщений
    /// \param[in] _msg Вещественное число – значение скорости
public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
        this->SetVelocity(_msg->data);
    }
    /// \brief Функция, работающая в потоке и обрабатывающая входящие сообщения
private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    /// \brief Установка скорости лазерного сканера
    /// \param[in] _vel Новая желаемая скорость
public: void SetVelocity(const double &_vel){
        this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), _vel);
    }

    /// \brief Обработка входящих сообщений
    /// \param[in] _msg Используется сообщение типа vector3
    /// но задействована только компонента x.
private: void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetVelocity(_msg->x());
    }


};
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin);
}
#endif
