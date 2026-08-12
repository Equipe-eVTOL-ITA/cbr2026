#pragma once

// Nó de visão da fase 1: cola entre o detector e a missão.
//
// Em 2025 este arquivo tinha 493 linhas, porque continha a matemática de
// projeção junto. Ela agora vive na biblioteca `vision_geometry`, testada sem
// ROS. O que sobra aqui é só o que é genuinamente da missão:
//
//   - assinar /base_detector/detections e guardar as caixas do último quadro;
//   - saber há quanto tempo foi a última detecção (a FSM usa para desistir);
//   - escolher a caixa mais próxima do centro da imagem;
//   - publicar as bases estimadas em /telemetry/bases para o RViz e o rosbag.
//
// A calibração vem de parâmetros ROS, e não de números no código, porque a
// câmera da simulação e a do drone são diferentes. Trocar de perfil é trocar de
// YAML.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <custom_msgs/msg/base_detection.hpp>

#include "vision_geometry/ground_projector.hpp"

class VisionFase1 : public rclcpp::Node
{
public:
  VisionFase1()
  : rclcpp::Node("fase1_vision")
  {
    // ── Intrínsecos ───────────────────────────────────────────────────────
    // Dois caminhos: fx/fy/cx/cy diretos (calibração real, do
    // camera_calibrator) ou FOV + resolução (o que o SDF do Gazebo dá). O
    // segundo só é usado quando `camera_fx` fica em zero.
    this->declare_parameter<double>("camera_fx", 0.0);
    this->declare_parameter<double>("camera_fy", 0.0);
    this->declare_parameter<double>("camera_cx", 0.0);
    this->declare_parameter<double>("camera_cy", 0.0);
    this->declare_parameter<double>("camera_hfov", 1.047);
    this->declare_parameter<int>("camera_width", 640);
    this->declare_parameter<int>("camera_height", 480);
    this->declare_parameter<int>("published_size", 0);

    // ── Extrínsecos ───────────────────────────────────────────────────────
    // R_b_c = Rz(yaw)·Ry(pitch)·Rx(roll) leva vetores da câmera para o corpo.
    //
    // O padrão yaw = pi/2 NÃO é arbitrário: é a montagem em que o topo da
    // imagem aponta para a frente do drone. Ver o README da vision_geometry —
    // extrínsecos nulos também dão uma câmera olhando para baixo, mas girada
    // 90°, e a diferença é silenciosa.
    this->declare_parameter<double>("camera_roll", 0.0);
    this->declare_parameter<double>("camera_pitch", 0.0);
    this->declare_parameter<double>("camera_yaw", M_PI / 2.0);
    this->declare_parameter<double>("camera_tx", 0.0);
    this->declare_parameter<double>("camera_ty", 0.0);
    this->declare_parameter<double>("camera_tz", 0.0);

    this->declare_parameter<double>("bbox_real_size", 1.0);

    // Guardas contra erro de escala do PnP. Ver o README da vision_geometry.
    // Uma base saindo do quadro encolhe na imagem, e o PnP le isso como
    // distancia maior -- foi assim que a missao perseguiu uma base estimada a
    // 10 m ABAIXO do solo para fora da arena.
    this->declare_parameter<double>("pnp_max_plane_deviation", 1.5);
    this->declare_parameter<double>("border_margin_px", 2.0);
    this->declare_parameter<bool>("bbox_is_normalized", true);
    this->declare_parameter<bool>("use_pnp", true);
    this->declare_parameter<double>("mean_base_height", -0.75);
    this->declare_parameter<std::string>("detection_topic", "/base_detector/detections");

    plane_z_ = this->get_parameter("mean_base_height").as_double();
    use_pnp_ = this->get_parameter("use_pnp").as_bool();

    projector_ = std::make_unique<vision_geometry::GroundProjector>(buildCalibration());

    const auto topic = this->get_parameter("detection_topic").as_string();
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&VisionFase1::onDetections, this, std::placeholders::_1));

    base_pub_ = this->create_publisher<custom_msgs::msg::BaseDetection>("/telemetry/bases", 10);

    // Sem detecção nenhuma ainda: a idade tem de começar grande, não em zero.
    // Zerar aqui faria a FSM acreditar, no primeiro ciclo, que acabou de ver
    // uma base.
    last_detection_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

    RCLCPP_INFO(this->get_logger(), "fase1_vision assinando %s", topic.c_str());
  }

  /// Segundos desde a última mensagem COM pelo menos uma detecção.
  double secondsSinceDetection() const
  {
    const std::chrono::duration<double> dt =
      std::chrono::steady_clock::now() - last_detection_;
    return dt.count();
  }

  bool hasDetection() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !boxes_.empty();
  }

  /// Cópia das caixas do último quadro.
  ///
  /// Cópia e não referência de propósito: o callback roda numa thread do
  /// executor e os estados leem noutra. Devolver referência para o vetor
  /// interno seria uma corrida de dados com o próximo quadro.
  std::vector<vision_geometry::BoundingBox> boxes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return boxes_;
  }

  /// Caixa mais próxima do centro da imagem.
  ///
  /// Com várias bases no campo de visão, a do centro é a que o drone está mais
  /// perto de sobrevoar, e é a que menos sofre com distorção de borda.
  bool closestBox(vision_geometry::BoundingBox & out) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (boxes_.empty()) return false;

    double best = std::numeric_limits<double>::max();
    for (const auto & b : boxes_) {
      const double d = std::hypot(b.center_x - 0.5, b.center_y - 0.5);
      if (d < best) {
        best = d;
        out = b;
      }
    }
    return true;
  }

  /// Projeta uma caixa para o mundo, em FRD.
  Eigen::Vector3d project(
    const Eigen::Vector3d & position,
    const Eigen::Vector3d & rpy,
    const vision_geometry::BoundingBox & box) const
  {
    vision_geometry::DronePose pose;
    pose.position = position;
    pose.rpy = rpy;

    return use_pnp_
           ? projector_->projectBySquarePnP(pose, box, plane_z_)
           : projector_->projectToPlane(pose, box, plane_z_);
  }

  /// Publica uma base para telemetria. `tipo` é "estimate", "first_estimate"
  /// ou "known" — o RViz colore por esse campo.
  void publishBase(const std::string & tipo, const Eigen::Vector3d & p)
  {
    custom_msgs::msg::BaseDetection msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.position.x = p.x();
    msg.position.y = p.y();
    msg.position.z = p.z();
    msg.base_type = tipo;
    msg.confidence = 1.0f;
    msg.detection_id = next_id_++;
    base_pub_->publish(msg);
  }

private:
  vision_geometry::CameraCalibration buildCalibration()
  {
    const double fx = this->get_parameter("camera_fx").as_double();
    const int width = this->get_parameter("camera_width").as_int();
    const int height = this->get_parameter("camera_height").as_int();

    vision_geometry::CameraCalibration calib;

    if (fx > 0.0) {
      calib.fx = fx;
      calib.fy = this->get_parameter("camera_fy").as_double();
      calib.cx = this->get_parameter("camera_cx").as_double();
      calib.cy = this->get_parameter("camera_cy").as_double();
      calib.image_width = width;
      calib.image_height = height;
    } else {
      // O camera_publisher recorta ao centro e redimensiona antes de publicar;
      // `published_size` é o lado da imagem que sai. fromHorizontalFov refaz
      // essa conta para que os intrínsecos correspondam à imagem que o
      // detector realmente viu, e não à do sensor.
      int published = this->get_parameter("published_size").as_int();
      if (published <= 0) published = std::min(width, height);

      calib = vision_geometry::CameraCalibration::fromHorizontalFov(
        this->get_parameter("camera_hfov").as_double(), width, height, published);
      RCLCPP_INFO(
        this->get_logger(),
        "camera_fx nao informado: intrinsecos derivados do FOV (fx=%.1f, %dx%d)",
        calib.fx, calib.image_width, calib.image_height);
    }

    calib.roll = this->get_parameter("camera_roll").as_double();
    calib.pitch = this->get_parameter("camera_pitch").as_double();
    calib.yaw = this->get_parameter("camera_yaw").as_double();
    calib.tx = this->get_parameter("camera_tx").as_double();
    calib.ty = this->get_parameter("camera_ty").as_double();
    calib.tz = this->get_parameter("camera_tz").as_double();
    calib.target_size = this->get_parameter("bbox_real_size").as_double();
    calib.pnp_max_plane_deviation =
      this->get_parameter("pnp_max_plane_deviation").as_double();
    calib.border_margin_px = this->get_parameter("border_margin_px").as_double();
    calib.bbox_is_normalized = this->get_parameter("bbox_is_normalized").as_bool();

    return calib;
  }

  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    std::vector<vision_geometry::BoundingBox> novas;
    novas.reserve(msg->detections.size());

    for (const auto & d : msg->detections) {
      vision_geometry::BoundingBox b;
      b.center_x = d.bbox.center.position.x;
      b.center_y = d.bbox.center.position.y;
      b.width = d.bbox.size_x;
      b.height = d.bbox.size_y;
      b.rotation = d.bbox.center.theta;
      novas.push_back(b);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes_ = std::move(novas);
    }

    // A idade só reinicia com detecção de verdade. Um quadro processado e
    // vazio não é notícia: é justamente o que o timeout precisa contar.
    if (!msg->detections.empty()) {
      last_detection_ = std::chrono::steady_clock::now();
    }
  }

  std::unique_ptr<vision_geometry::GroundProjector> projector_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<custom_msgs::msg::BaseDetection>::SharedPtr base_pub_;

  mutable std::mutex mutex_;
  std::vector<vision_geometry::BoundingBox> boxes_;
  std::chrono::steady_clock::time_point last_detection_;

  double plane_z_{-0.75};
  bool use_pnp_{true};
  uint32_t next_id_{0};
};
