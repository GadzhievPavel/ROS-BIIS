#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// Поскольку программный интерфейс Gazebo менялся в разных версиях
// То указываем различные варианты с помощью макросов #if..#endif.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif
/////////////////////////////////////////////////
int main(int _argc, char **_argv){
// Загружаем симулятор как клиент
#if GAZEBO_MAJOR_VERSION < 6
gazebo::setupClient(_argc, _argv);
#else
gazebo::client::setup(_argc, _argv);
#endif
// Создаём узел
gazebo::transport::NodePtr node(new
gazebo::transport::Node());
node->Init();
// Публикатор
gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");
// Ожидаем, пока подписчик не подключится к публикатору
pub->WaitForConnection();
// Создаём сообщение в формате vector3
gazebo::msgs::Vector3d msg;
// Устанавливаем желаемую скорость в компоненту x
#if GAZEBO_MAJOR_VERSION < 6
gazebo::msgs::Set(&msg,gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
#else
gazebo::msgs::Set(&msg,ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
#endif
// Отправляем сообщение
pub->Publish(msg);
// Завершаем работу.
#if GAZEBO_MAJOR_VERSION < 6
gazebo::shutdown();
#else
gazebo::client::shutdown();
#endif
}
