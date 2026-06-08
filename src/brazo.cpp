// Nodo "brazo" (C++ con MoveIt2 / MoveGroupInterface).
//
// Ofrece /pick_place (std_srvs/Trigger): planifica y ejecuta la recogida/colocación del UR5e en
// ESPACIO DE JOINTS, controla el gripper Robotiq por FollowJointTrajectory y publica /agarre para
// que el plugin del Supervisor pegue/suelte la caja. Grupo de planificación: "ur5e_arm".
//
// POR QUÉ ESPACIO DE JOINTS (no IK de pose por waypoint):
//   Planificar a una POSE cartesiana por waypoint hacía que KDL devolviera soluciones en ramas
//   distintas (a veces el brazo alcanzaba "hacia atrás" sobre el pedestal) -> el brazo se ENROLLABA,
//   giraba sobre su eje y chocaba con el pedestal/consigo mismo (cientos de contactos en Webots).
//   En cambio, las 5 configuraciones de joints de abajo se calcularon UNA vez con /compute_ik
//   (semilla hacia adelante, rama consistente: wrist_2=-1.571 en todas) para poses top-down sobre la
//   caja (en frente del robot) y sobre el sitio de descarga. Ejecutar en joints = interpolación
//   directa y predecible: el descenso pick<-hover_pick cambia SOLO lift/elbow/wrist_1 (baja recto), y
//   pick<->place es un giro de pan de 90°. Como el agarre es por teletransporte (el plugin captura el
//   offset caja<-tool0 al cerrar), no hace falta alineación cartesiana exacta.
//
//   El brazo se SPAWNEA ya en `hover_pick` (init_pos del URDFSpawner) -> sin el gran despliegue desde
//   "pointed_up". home == hover_pick.
//
// Afinables en vivo:  ros2 param set /brazo pick "[...6 joints...]"  + ros2 service call /pick_place ...

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Orden de joints del grupo "ur5e_arm" (debe coincidir con el SRDF/controlador).
static const std::vector<std::string> JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

class Brazo {
public:
  Brazo(rclcpp::Node::SharedPtr node, std::shared_ptr<MoveGroupInterface> mgi)
      : node_(node), mgi_(mgi) {
    double vel = 0.0;
    node_->get_parameter_or<double>("vel_scaling", vel, 0.15);
    mgi_->setMaxVelocityScalingFactor(vel);
    mgi_->setMaxAccelerationScalingFactor(vel);
    mgi_->setPlanningTime(10.0);
    mgi_->setNumPlanningAttempts(8);

    // Configuraciones de joints (rad), orden de JOINTS. Calculadas con /compute_ik para poses
    // top-down sobre la caja (en frente del robot) y el sitio de descarga; rama consistente.
    hover_pick_  = readJoints("hover_pick",  {-0.3007, -1.6833, 1.9549, -1.8424, -1.5708, -1.8715});
    pick_        = readJoints("pick",        {-0.3007, -1.4919, 2.2051, -2.2839, -1.5708, -1.8715});
    hover_place_ = readJoints("hover_place", { 1.2701, -1.6596, 2.0143, -1.9255, -1.5708, -0.3007});
    place_       = readJoints("place",       { 1.2701, -1.4307, 2.2395, -2.3796, -1.5708, -0.3007});
    home_        = readJoints("home",        hover_pick_);  // home == hover_pick

    agarre_pub_ = node_->create_publisher<std_msgs::msg::String>("/agarre", 10);
    grip_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        node_, "/ur5e/ur_joint_trajectory_controller/follow_joint_trajectory");

    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_ = node_->create_service<Trigger>(
        "/pick_place",
        std::bind(&Brazo::onPickPlace, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), cb_group_);

    RCLCPP_INFO(node_->get_logger(),
                "brazo listo (grupo=ur5e_arm, espacio de joints, servicio /pick_place)");
  }

private:
  std::vector<double> readJoints(const std::string &name, std::vector<double> def) {
    std::vector<double> v;
    node_->get_parameter_or<std::vector<double>>(name, v, def);
    if (v.size() == 6) return v;
    RCLCPP_WARN(node_->get_logger(), "param '%s' no tiene 6 valores; uso default", name.c_str());
    return def;
  }

  // Planifica+ejecuta a una configuración de joints (interpolación directa, sin IK por waypoint).
  bool moveJ(const std::vector<double> &cfg, const char *label) {
    mgi_->setStartStateToCurrentState();
    mgi_->setJointValueTarget(JOINTS, cfg);
    const bool ok = (mgi_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) RCLCPP_WARN(node_->get_logger(), "MoveIt falló en: %s", label);
    return ok;
  }

  // Gripper: goal PARCIAL (solo los 3 dedos) al ur_joint_trajectory_controller.
  void gripper(double f1, double f2, double fm) {
    if (!grip_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(node_->get_logger(), "gripper: action server no disponible");
      return;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {f1, f2, fm};
    pt.time_from_start = rclcpp::Duration::from_seconds(1.0);
    goal.trajectory.points.push_back(pt);
    grip_client_->async_send_goal(goal);
    std::this_thread::sleep_for(1500ms);
  }
  void abrirGripper()  { gripper(0.05, 0.05, 0.05); }
  void cerrarGripper() { gripper(0.70, 0.70, 0.60); }  // < 1.2218 (límite del dedo)
  void agarre(const std::string &cmd) {
    std_msgs::msg::String m;
    m.data = cmd;
    agarre_pub_->publish(m);
  }

  void onPickPlace(const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> res) {
    RCLCPP_INFO(node_->get_logger(), "pick&place: inicio");
    abrirGripper();
    if (!moveJ(hover_pick_, "hover_pick")) return fallo(res, "hover_pick");
    if (!moveJ(pick_, "pick"))             return fallo(res, "pick");
    cerrarGripper();
    agarre("attach");                                  // captura offset y pega la caja a tool0
    if (!moveJ(hover_pick_, "lift"))       return fallo(res, "lift");
    if (!moveJ(hover_place_, "hover_place")) return fallo(res, "hover_place");
    if (!moveJ(place_, "place"))           return fallo(res, "place");
    abrirGripper();
    agarre("to_turtlebot");                            // suelta (E2: queda en sitio; E3: TurtleBot)
    if (!moveJ(hover_place_, "retreat"))   return fallo(res, "retreat");
    moveJ(home_, "home");                              // vuelve a hover_pick (no crítico si falla)
    res->success = true;
    res->message = "pick&place OK";
    RCLCPP_INFO(node_->get_logger(), "pick&place: OK");
  }
  void fallo(std::shared_ptr<Trigger::Response> res, const std::string &donde) {
    res->success = false;
    res->message = "fallo en " + donde;
    RCLCPP_WARN(node_->get_logger(), "pick&place: %s", res->message.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> mgi_;
  std::vector<double> hover_pick_, pick_, hover_place_, place_, home_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr agarre_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr grip_client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Service<Trigger>::SharedPtr srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "brazo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  auto mgi = std::make_shared<MoveGroupInterface>(node, "ur5e_arm");
  Brazo brazo(node, mgi);

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
