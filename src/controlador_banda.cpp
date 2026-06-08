// Plugin C++ de webots_ros2_driver (corre en el Robot supervisor del mundo).
//
//  - Banda:  /conveyor/enable (Bool) -> muta el campo `speed` de DEF CONVEYOR_BELT (visual) y
//            marca belt_on_ (mueve la caja roja CINEMATICA por la banda).
//  - Caja roja CINEMATICA (DEF BOX_ROJA, sin physics ni colisión): el plugin es su único motor.
//      ON_BELT  : con la banda en marcha, avanza +X a belt_speed (sustituye la fricción de la
//                 banda; al ser cinemática no puede colapsar el solver al agarrarla).
//      GRIP     : tras "attach" sigue al gripper RÍGIDA. Captura UNA vez el offset caja<-tool0
//                 (en el frame de tool0) y lo mantiene -> la caja se queda DONDE estaba al cerrar,
//                 sin "aparecer de la nada"; al moverse/levantarse tool0, la caja va con él.
//      RIDE_TB  : tras "to_turtlebot" viaja ENCIMA del TurtleBot (sigue DEF TURTLEBOT por Supervisor,
//                 centrada + ride_z en Z) -> la caja llega a la estación montada en el robot móvil.
//      DELIVERED: tras "release" queda quieta donde se soltó.
//  - Publica /caja_roja/pos (PointStamped, world): el nodo `brazo` planifica el pick a ESA posición
//    real (por IK), así el gripper baja sobre la caja en la banda de verdad.
//
// El robot tiene `supervisor TRUE` -> API wb_supervisor_*. WebotsNode es un rclcpp::Node (creo aquí
// subs/pubs y un TF listener); el driver hace el spin y robot.step() (en step() muevo la caja).

#include <memory>
#include <string>
#include <unordered_map>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/supervisor.h>
#include <webots/robot.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <pluginlib/class_list_macros.hpp>

namespace clasificador_rojo {

static double paramD(const std::unordered_map<std::string, std::string> &p,
                     const std::string &k, double def) {
  auto it = p.find(k);
  if (it == p.end()) return def;
  try { return std::stod(it->second); } catch (...) { return def; }
}
static std::string paramS(const std::unordered_map<std::string, std::string> &p,
                          const std::string &k, const std::string &def) {
  auto it = p.find(k);
  return it == p.end() ? def : it->second;
}

class ControladorBanda : public webots_ros2_driver::PluginInterface {
public:
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override {
    node_ = node;
    belt_speed_ = paramD(parameters, "belt_speed", 0.25);
    world_frame_ = paramS(parameters, "world_frame", "world");
    grip_frame_ = paramS(parameters, "grip_frame", "tool0");
    ride_z_ = paramD(parameters, "ride_z", 0.15);  // altura de la caja sobre el TurtleBot

    dt_ = wb_robot_get_basic_time_step() / 1000.0;
    if (dt_ <= 0.0) dt_ = 0.016;

    WbNodeRef belt = wb_supervisor_node_get_from_def("CONVEYOR_BELT");
    if (belt) belt_speed_field_ = wb_supervisor_node_get_field(belt, "speed");
    else RCLCPP_WARN(node_->get_logger(), "[controlador_banda] falta DEF CONVEYOR_BELT");

    box_ = wb_supervisor_node_get_from_def("BOX_ROJA");
    if (box_) box_tr_ = wb_supervisor_node_get_field(box_, "translation");
    else RCLCPP_WARN(node_->get_logger(), "[controlador_banda] falta DEF BOX_ROJA");

    // DEF TURTLEBOT: la caja lo seguirá (RIDE_TB) para viajar a la estación montada encima.
    tb_ = wb_supervisor_node_get_from_def("TURTLEBOT");
    if (tb_) tb_tr_ = wb_supervisor_node_get_field(tb_, "translation");
    else RCLCPP_WARN(node_->get_logger(), "[controlador_banda] falta DEF TURTLEBOT (caja no viajará)");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pos_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/caja_roja/pos", rclcpp::QoS(1));
    enable_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/conveyor/enable", rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::SharedPtr m) { setBelt(m->data); });
    grasp_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/agarre", rclcpp::QoS(1),
        [this](const std_msgs::msg::String::SharedPtr m) { onGrasp(m->data); });

    RCLCPP_INFO(node_->get_logger(),
                "[controlador_banda] listo (belt_speed=%.3f, world=%s, grip=%s, dt=%.3f)",
                belt_speed_, world_frame_.c_str(), grip_frame_.c_str(), dt_);
  }

  void step() override {
    if (!box_tr_) return;

    // Publica la posición de la caja (world) para que el brazo planifique el pick por IK.
    const double *cur = wb_supervisor_field_get_sf_vec3f(box_tr_);
    geometry_msgs::msg::PointStamped ps;
    ps.header.frame_id = world_frame_;
    ps.point.x = cur[0]; ps.point.y = cur[1]; ps.point.z = cur[2];
    pos_pub_->publish(ps);

    if (box_state_ == Box::ON_BELT) {
      if (belt_on_) {  // avanza la caja por la banda (cinemática)
        const double np[3] = {cur[0] + belt_speed_ * dt_, cur[1], cur[2]};
        wb_supervisor_field_set_sf_vec3f(box_tr_, np);
      }
      return;
    }

    if (box_state_ == Box::GRIP) {  // sigue al gripper, rígida, con el offset capturado al cerrar
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(world_frame_, grip_frame_, tf2::TimePointZero);
      } catch (const std::exception &e) {
        if (!warned_) {
          RCLCPP_WARN(node_->get_logger(), "[agarre] TF %s->%s no disponible: %s",
                      world_frame_.c_str(), grip_frame_.c_str(), e.what());
          warned_ = true;
        }
        return;
      }
      warned_ = false;
      const tf2::Transform T(
          tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w),
          tf2::Vector3(tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z));
      if (!have_offset_) {  // captura UNA vez dónde está la caja respecto a tool0 (sin salto)
        box_local_ = T.inverse() * tf2::Vector3(cur[0], cur[1], cur[2]);
        have_offset_ = true;
      }
      const tf2::Vector3 pw = T * box_local_;
      const double np[3] = {pw.x(), pw.y(), pw.z()};
      wb_supervisor_field_set_sf_vec3f(box_tr_, np);
      return;
    }

    if (box_state_ == Box::RIDE_TB) {  // viaja centrada encima del TurtleBot (lo sigue por Supervisor)
      if (tb_tr_) {
        const double *t = wb_supervisor_field_get_sf_vec3f(tb_tr_);
        const double np[3] = {t[0], t[1], t[2] + ride_z_};
        wb_supervisor_field_set_sf_vec3f(box_tr_, np);
      }
      return;
    }
    // DELIVERED: no se toca (queda donde se soltó).
  }

private:
  enum class Box { ON_BELT, GRIP, RIDE_TB, DELIVERED };

  void setBelt(bool on) {
    belt_on_ = on;
    if (belt_speed_field_)
      wb_supervisor_field_set_sf_float(belt_speed_field_, on ? belt_speed_ : 0.0);
  }
  void onGrasp(const std::string &c) {
    if (c == "attach") { box_state_ = Box::GRIP; have_offset_ = false; }
    else if (c == "to_turtlebot") { box_state_ = Box::RIDE_TB; }  // a viajar encima del TurtleBot
    else if (c == "release") { box_state_ = Box::DELIVERED; }
    RCLCPP_INFO(node_->get_logger(), "[agarre] %s -> estado caja", c.c_str());
  }

  webots_ros2_driver::WebotsNode *node_{nullptr};
  WbFieldRef belt_speed_field_{0};
  WbNodeRef box_{0};
  WbFieldRef box_tr_{0};
  WbNodeRef tb_{0};
  WbFieldRef tb_tr_{0};
  double belt_speed_{0.25}, dt_{0.016}, ride_z_{0.15};
  std::string world_frame_, grip_frame_;
  Box box_state_{Box::ON_BELT};
  bool belt_on_{false}, have_offset_{false}, warned_{false};
  tf2::Vector3 box_local_{0, 0, 0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_sub_;
};

}  // namespace clasificador_rojo

PLUGINLIB_EXPORT_CLASS(clasificador_rojo::ControladorBanda, webots_ros2_driver::PluginInterface)
