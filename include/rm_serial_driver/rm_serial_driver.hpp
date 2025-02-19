#pragma once

#include <serial/serial.h>

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "rm_msgs/msg/data_mcu.hpp"
#include "rm_msgs/msg/data_ref.hpp"
#include "rm_msgs/msg/data_ai.hpp" 
#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/protocol.hpp"

namespace rm_serial_driver
{

  class RMSerialDriver : public rclcpp::Node
  {
  public:
    explicit RMSerialDriver(const rclcpp::NodeOptions &options);
    ~RMSerialDriver() override;

  private:
    size_t get_expected_length(uint8_t id);
    void process_packet(uint8_t id, const char *data, size_t packet_size);
    void receive_data();
    void reopen_port();
    void data_ai_callback(const rm_msgs::msg::DataAI::SharedPtr msg);
    
    std::unique_ptr<serial::Serial> serial_port_;
    rclcpp::Publisher<rm_msgs::msg::DataMCU>::SharedPtr data_mcu_pub_;
    rclcpp::Publisher<rm_msgs::msg::DataRef>::SharedPtr data_ref_pub_;
    rclcpp::Subscription<rm_msgs::msg::DataAI>::SharedPtr data_ai_sub_; 
    std::thread read_thread_;   // 串口数据读取线程
    std::atomic<bool> running_; // 原子操作: 保证多线程操作时的数据一致性
    std::string serial_buffer_; // 串口数据缓冲区
  };

} // namespace rm_serial_driver
