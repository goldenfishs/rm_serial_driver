#include "rm_serial_driver/rm_serial_driver.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace rm_serial_driver
{

  /**
   * @brief 构造函数：初始化节点，打开串口，创建发布者和订阅者，启动数据接收线程，打印启动信息
   * @param options
   */
  RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
      : Node("rm_serial_driver", options), running_(true)
  {
    std::string device = this->declare_parameter("device_name", std::string("/dev/ttyACM0"));
    int baud = this->declare_parameter("baud_rate", 115200);
    serial_port_ =
        std::make_unique<serial::Serial>(device, baud, serial::Timeout::simpleTimeout(1000));
    data_mcu_pub_ = this->create_publisher<rm_msgs::msg::DataMCU>("data_mcu", 10);
    data_ref_pub_ = this->create_publisher<rm_msgs::msg::DataRef>("data_ref", 10);
    data_ai_sub_ = this->create_subscription<rm_msgs::msg::DataAI>(
        "data_ai", 10, std::bind(&RMSerialDriver::data_ai_callback, this, std::placeholders::_1)); // 新增订阅者
    read_thread_ = std::thread(&RMSerialDriver::receive_data, this);
    RCLCPP_INFO(get_logger(), "RM 启动！！！");
  }

  /**
   * @brief 析构函数：关闭串口，等待数据接收线程结束
   */
  RMSerialDriver::~RMSerialDriver()
  {
    running_.store(false);
    if (read_thread_.joinable())
    {
      read_thread_.join();
    }
    if (serial_port_->isOpen())
    {
      serial_port_->close();
    }
  }

  /**
   * @brief AI 数据回调函数
   * @param msg
   */
  void RMSerialDriver::data_ai_callback(const rm_msgs::msg::DataAI::SharedPtr msg)
  {
    PackageAI_t package;
    std::memcpy(&package.data, msg.get(), sizeof(DataAI_t));
    package.crc16 = crc16::CRC16_Calc(reinterpret_cast<const uint8_t *>(&package.data), sizeof(DataAI_t), CRC16_INIT);
    serial_port_->write(reinterpret_cast<const uint8_t *>(&package), sizeof(PackageAI_t));
  }

  /**
   * @brief 获取期望的数据包长度
   * @param id
   * @return
   */
  size_t RMSerialDriver::get_expected_length(uint8_t id)
  {
    if (id == ID_MCU)
      return sizeof(PackageMCU_t);
    else if (id == ID_REF)
      return sizeof(PackageReferee_t);
    return 0;
  }

  /**
   * @brief 处理接收到的数据包
   * @param id
   * @param data
   * @param packet_size
   */
  void RMSerialDriver::process_packet(uint8_t id, const char *data, size_t packet_size)
  {
    if (id == ID_MCU)
    {
      PackageMCU_t mcu;
      std::memcpy(&mcu, data, packet_size); // 将 data 拷贝到 mcu 中
      if (crc16::CRC16_Verify(reinterpret_cast<uint8_t *>(&mcu) + 1, packet_size - 1))
      {
        rm_msgs::msg::DataMCU msg;
        std::memcpy(&msg, &mcu.data, sizeof(mcu.data)); // 将数据拷贝到 msg 中
        data_mcu_pub_->publish(msg);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "CRC 校验失败(MCU)");
      }
    }
    else if (id == ID_REF)
    {
      PackageReferee_t ref;
      std::memcpy(&ref, data, packet_size);
      if (crc16::CRC16_Verify(reinterpret_cast<uint8_t *>(&ref) + 1, packet_size - 1))
      {
        rm_msgs::msg::DataRef msg;
        std::memcpy(&msg, &ref.data, sizeof(ref.data));
        data_ref_pub_->publish(msg);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "CRC 校验失败(MCU)");
      }
    }
    else
    {
      // 未知 ID，丢弃该字节
    }
  }

  /**
   * @brief 串口数据接收线程
   * @param args 线程参数
   */
  void RMSerialDriver::receive_data()
  {
    while (rclcpp::ok() && running_.load())
    {
      try
      {
        if (serial_port_->available())
        {
          std::string buffer = serial_port_->read(serial_port_->available());
          serial_buffer_ += buffer;

          while (true)
          {
            size_t header_pos = serial_buffer_.find_first_of(
                std::string() + static_cast<char>(ID_MCU) + static_cast<char>(ID_REF)); // 查找第一个出现的字符
            if (header_pos == std::string::npos)
            {
              serial_buffer_.clear();
              break;
            }
            if (header_pos > 0)
            {
              serial_buffer_.erase(0, header_pos);
            }
            if (serial_buffer_.empty())
              break;

            uint8_t id = static_cast<uint8_t>(serial_buffer_[0]);
            size_t expected_length = get_expected_length(id);
            if (expected_length == 0)
            {
              serial_buffer_.erase(0, 1);
              continue;
            }
            if (serial_buffer_.size() < expected_length)
              break;

            process_packet(id, serial_buffer_.data(), expected_length);
            serial_buffer_.erase(0, expected_length);
          }
        }
      }
      catch (const std::exception &ex)
      {
        reopen_port();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /**
   * @brief 重启串口
   * @note 当串口意外关闭时，尝试重新打开串口
   */
  void RMSerialDriver::reopen_port()
  {
    if (serial_port_->isOpen())
    {
      serial_port_->close();
    }

    bool port_opened = false;
    while (!port_opened && rclcpp::ok())
    {
      try
      {
        RCLCPP_WARN(get_logger(), "串口意外关闭，正在尝试重新打开...");
        serial_port_->open();
        port_opened = true;
      }
      catch (const std::exception &ex)
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)