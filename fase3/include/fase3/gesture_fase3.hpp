#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/msg/gesture.hpp>
#include <custom_msgs/msg/hand_location.hpp>

class GestureFase3 : public rclcpp::Node
{
public:
  GestureFase3()
  : rclcpp::Node("fase3_gesture")
  {
    this->declare_parameter<std::string>("gesture_topic", "/gesture_detector/gestures");
    this->declare_parameter<std::string>("hand_topic", "/gesture_detector/hand_location");

    this->declare_parameter<int>("controlling_hand", 0);

    hand_index_ = this->get_parameter("controlling_hand").as_int();

    const auto gesture_topic = this->get_parameter("gesture_topic").as_string();
    const auto hand_topic = this->get_parameter("hand_topic").as_string();

    gesture_sub_ = this->create_subscription<custom_msgs::msg::Gesture>(
      gesture_topic, rclcpp::SensorDataQoS(),
      std::bind(&GestureFase3::onGestures, this, std::placeholders::_1));

    hand_sub_ = this->create_subscription<custom_msgs::msg::HandLocation>(
      hand_topic, rclcpp::SensorDataQoS(),
      std::bind(&GestureFase3::onHand, this, std::placeholders::_1));

    // Sem detecção ainda: a idade tem de começar GRANDE, não em zero. Zerar
    // aqui faria a FSM acreditar, no primeiro ciclo, que acabou de ver uma mão.
    const auto ha_muito_tempo = std::chrono::steady_clock::now() - std::chrono::hours(1);
    ultimo_gesto_ = ha_muito_tempo;
    ultima_mao_ = ha_muito_tempo;

    RCLCPP_INFO(
      this->get_logger(), "fase3_gesture assinando %s e %s (mao de controle: %d)",
      gesture_topic.c_str(), hand_topic.c_str(), hand_index_);
  }

  // Gesto da mão de controle, ou "" se não há.
  std::string gesture() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hand_index_ < 0 || static_cast<size_t>(hand_index_) >= gestures_.size()) {
      return "";
    }
    return gestures_[hand_index_];
  }

  // Segundos desde a última mensagem de gesto com a mão de controle presente.
  double secondsSinceGesture() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::chrono::duration<double> dt =
      std::chrono::steady_clock::now() - ultimo_gesto_;
    return dt.count();
  }

  // Segundos desde a última posição de mão recebida.
  double secondsSinceHand() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::chrono::duration<double> dt =
      std::chrono::steady_clock::now() - ultima_mao_;
    return dt.count();
  }

  std::pair<float, float> hand() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {hand_x_, hand_y_};
  }

private:
  void onGestures(const custom_msgs::msg::Gesture::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gestures_ = msg->gestures;

    if (hand_index_ >= 0 &&
      static_cast<size_t>(hand_index_) < gestures_.size() &&
      !gestures_[hand_index_].empty())
    {
      ultimo_gesto_ = std::chrono::steady_clock::now();
    }
  }

  void onHand(const custom_msgs::msg::HandLocation::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hand_x_ = msg->hand_x;
    hand_y_ = msg->hand_y;
    ultima_mao_ = std::chrono::steady_clock::now();
  }

  rclcpp::Subscription<custom_msgs::msg::Gesture>::SharedPtr gesture_sub_;
  rclcpp::Subscription<custom_msgs::msg::HandLocation>::SharedPtr hand_sub_;

  mutable std::mutex mutex_;
  std::vector<std::string> gestures_;
  float hand_x_{0.5f};
  float hand_y_{0.5f};
  std::chrono::steady_clock::time_point ultimo_gesto_;
  std::chrono::steady_clock::time_point ultima_mao_;

  int hand_index_{0};
};
