// Orquestador (C++): máquina de estados del proceso.
//
// Entradas:  /caja_roja (Bool)        de deteccion_rojo
//            /banda/comando (String)  de mqtt_client  ("start"/"stop")
// Salidas:   /conveyor/enable (Bool)  al plugin de la banda  (ÚNICO publicador)
// Clientes:  /pick_place (std_srvs/Trigger)        -> nodo brazo  (Etapa 2)
//            /navigate_to_pose (nav2_msgs action)  -> Nav2         (Etapa 3)
//
// El parámetro `etapa` (1/2/3) decide hasta dónde llega el ciclo:
//   Etapa 1: caja roja -> detiene banda (reanuda con "start" por MQTT).
//   Etapa 2: + pick&place con el UR5e, luego reanuda la banda.
//   Etapa 3: + lleva la caja a la estación con el TurtleBot, luego reanuda la banda.

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Trigger = std_srvs::srv::Trigger;

class Orquestador : public rclcpp::Node {
public:
  Orquestador() : rclcpp::Node("orquestador") {
    etapa_ = declare_parameter<int>("etapa", 1);
    frames_min_ = declare_parameter<int>("frames_min", 3);
    est_x_ = declare_parameter<double>("estacion_x", -2.0);
    est_y_ = declare_parameter<double>("estacion_y", -1.5);
    est_yaw_ = declare_parameter<double>("estacion_yaw", 0.0);

    pub_banda_ = create_publisher<std_msgs::msg::Bool>("/conveyor/enable", 10);
    sub_caja_ = create_subscription<std_msgs::msg::Bool>(
        "/caja_roja", 10, std::bind(&Orquestador::onCaja, this, std::placeholders::_1));
    sub_cmd_ = create_subscription<std_msgs::msg::String>(
        "/banda/comando", 10, std::bind(&Orquestador::onComando, this, std::placeholders::_1));

    cli_pick_ = create_client<Trigger>("/pick_place");
    cli_nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    // Arranca la banda tras 2 s. En Etapa 2/3 ESPERA primero a que el brazo (/pick_place) esté
    // listo: move_group tarda ~10 s en inicializar; sin esto la caja llega a la cámara antes de
    // que el brazo arranque (carrera de arranque) y se pasa de largo sin que nadie la recoja.
    timer_init_ = create_wall_timer(2s, [this]() {
      timer_init_->cancel();
      if (etapa_ >= 2) {
        RCLCPP_INFO(get_logger(), "Etapa %d: esperando a que el brazo (/pick_place) esté listo...", etapa_);
        if (!cli_pick_->wait_for_service(120s))
          RCLCPP_WARN(get_logger(), "/pick_place no apareció en 120 s; arranco la banda igualmente");
        else
          RCLCPP_INFO(get_logger(), "brazo listo; arranco la banda");
      }
      setBanda(true);
      estado_ = "BANDA_ON";
      RCLCPP_INFO(get_logger(), "Orquestador (etapa %d): banda en marcha", etapa_);
    });
  }

private:
  void setBanda(bool on) {
    std_msgs::msg::Bool b;
    b.data = on;
    pub_banda_->publish(b);
  }

  void onComando(const std_msgs::msg::String::SharedPtr m) {
    const std::string c = m->data;
    if (c == "stop" || c == "STOP" || c == "parar") {
      manual_stop_ = true;
      setBanda(false);
      estado_ = "DETENIDA_MANUAL";
      RCLCPP_INFO(get_logger(), "MQTT: stop (banda detenida)");
    } else if (c == "start" || c == "START" || c == "arrancar") {
      manual_stop_ = false;
      // No reanudar si hay un pick&place/navegación en curso (Etapa 2/3).
      if (etapa_ >= 2 && estado_ == "PROCESANDO") {
        RCLCPP_INFO(get_logger(), "MQTT: start (se reanudará al terminar el ciclo)");
      } else {
        caja_frames_ = 0;
        setBanda(true);
        estado_ = "BANDA_ON";
        RCLCPP_INFO(get_logger(), "MQTT: start (banda en marcha)");
      }
    }
  }

  void onCaja(const std_msgs::msg::Bool::SharedPtr m) {
    caja_frames_ = m->data ? (caja_frames_ + 1) : 0;
    if (estado_ != "BANDA_ON" || manual_stop_) return;
    if (caja_frames_ < frames_min_) return;

    RCLCPP_INFO(get_logger(), "Caja roja confirmada -> detengo la banda");
    setBanda(false);
    estado_ = "PROCESANDO";
    if (etapa_ >= 2) {
      iniciarPickPlace();
    }
    // Etapa 1: permanece detenida hasta recibir "start" por MQTT.
  }

  void iniciarPickPlace() {
    if (!cli_pick_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Servicio /pick_place no disponible; reanudo la banda");
      finProceso();
      return;
    }
    auto req = std::make_shared<Trigger::Request>();
    cli_pick_->async_send_request(
        req, [this](rclcpp::Client<Trigger>::SharedFuture fut) {
          auto res = fut.get();
          if (!res->success) {
            RCLCPP_WARN(get_logger(), "pick&place falló: %s", res->message.c_str());
            finProceso();
            return;
          }
          RCLCPP_INFO(get_logger(), "pick&place OK");
          if (etapa_ >= 3) {
            enviarNavegacion();
          } else {
            finProceso();
          }
        });
  }

  void enviarNavegacion() {
    if (!cli_nav_->wait_for_action_server(2s)) {
      RCLCPP_WARN(get_logger(), "Nav2 no disponible; reanudo la banda");
      finProceso();
      return;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = est_x_;
    goal.pose.pose.position.y = est_y_;
    goal.pose.pose.orientation.z = std::sin(est_yaw_ / 2.0);
    goal.pose.pose.orientation.w = std::cos(est_yaw_ / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "TurtleBot llegó a la estación");
          } else {
            RCLCPP_WARN(get_logger(), "Nav2 no completó (code=%d)", static_cast<int>(result.code));
          }
          finProceso();
        };
    cli_nav_->async_send_goal(goal, opts);
    RCLCPP_INFO(get_logger(), "TurtleBot -> estación (%.2f, %.2f)", est_x_, est_y_);
  }

  void finProceso() {
    caja_frames_ = 0;
    // Una vez COLOCADA la pieza en su posición final, la banda queda DETENIDA (no se reanuda sola).
    // Para volver a arrancarla, el operador manda "start" por MQTT.
    setBanda(false);
    estado_ = manual_stop_ ? "DETENIDA_MANUAL" : "COMPLETADO";
    RCLCPP_INFO(get_logger(),
                "Pieza colocada en posición final -> banda DETENIDA ('start' por MQTT para reanudar)");
  }

  int etapa_{1};
  int frames_min_{3};
  int caja_frames_{0};
  double est_x_{-2.0}, est_y_{-1.5}, est_yaw_{0.0};
  bool manual_stop_{false};
  std::string estado_{"ARRANCANDO"};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_banda_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_caja_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
  rclcpp::Client<Trigger>::SharedPtr cli_pick_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr cli_nav_;
  rclcpp::TimerBase::SharedPtr timer_init_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Orquestador>());
  rclcpp::shutdown();
  return 0;
}
