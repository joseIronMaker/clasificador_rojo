// Detección de cajas rojas (C++ + OpenCV).
//
// Se suscribe a la cámara de Webots (/camera/image_color, BGRA8), segmenta el rojo en HSV
// (dos rangos, porque el rojo está en ambos extremos del matiz) y publica /caja_roja (Bool)
// = true cuando hay un contorno rojo con área suficiente. Publica además /caja_roja/centro
// (Point: x, y en píxeles; z = área) para depuración/centrado.
//
// En C++ para minimizar el coste por frame (sin GIL, OpenCV nativo).

#include <algorithm>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

class DeteccionRojo : public rclcpp::Node {
public:
  DeteccionRojo() : rclcpp::Node("deteccion_rojo") {
    area_min_ = this->declare_parameter<int>("area_min", 1200);
    const std::string image_topic =
        this->declare_parameter<std::string>("image_topic", "/camera/image_color");

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&DeteccionRojo::onImage, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Bool>("/caja_roja", 10);
    pub_centro_ = this->create_publisher<geometry_msgs::msg::Point>("/caja_roja/centro", 10);
    // Imagen ANOTADA (caja resaltada + estado) para verla en RViz / rqt_image_view.
    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>("/deteccion/imagen", 10);

    RCLCPP_INFO(get_logger(), "deteccion_rojo iniciado (topic=%s, area_min=%d)",
                image_topic.c_str(), area_min_);
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat bgr;
    try {
      // La cámara de Webots publica BGRA8; cv_bridge convierte a BGR8.
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    if (bgr.empty()) return;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat m1, m2, mask;
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), m1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Point2f best_c(0, 0);
    int best_idx = -1;
    bool found = false;
    for (size_t i = 0; i < contours.size(); ++i) {
      const double a = cv::contourArea(contours[i]);
      if (a >= area_min_ && a > best_area) {
        best_area = a;
        best_idx = static_cast<int>(i);
        found = true;
        const cv::Moments mu = cv::moments(contours[i]);
        if (mu.m00 > 0) best_c = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
      }
    }

    std_msgs::msg::Bool out;
    out.data = found;
    pub_->publish(out);
    if (found) {
      geometry_msgs::msg::Point p;
      p.x = best_c.x;
      p.y = best_c.y;
      p.z = best_area;
      pub_centro_->publish(p);
    }

    // Imagen anotada para visualización: recuadro + centro + estado sobre el frame de la cámara.
    if (pub_img_->get_subscription_count() > 0) {
      cv::Mat anot = bgr.clone();
      const cv::Scalar verde(0, 255, 0), rojo(0, 0, 255);
      if (found && best_idx >= 0) {
        const cv::Rect r = cv::boundingRect(contours[best_idx]);
        cv::rectangle(anot, r, verde, 2);
        cv::circle(anot, best_c, 4, verde, -1);
        cv::putText(anot, "CAJA ROJA", cv::Point(r.x, std::max(0, r.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, verde, 2);
      }
      cv::putText(anot, found ? "DETECTADA -> banda STOP" : "buscando...",
                  cv::Point(6, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, found ? verde : rojo, 1);
      auto im = cv_bridge::CvImage(msg->header, "bgr8", anot).toImageMsg();
      pub_img_->publish(*im);
    }
  }

  int area_min_{1200};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_centro_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeteccionRojo>());
  rclcpp::shutdown();
  return 0;
}
