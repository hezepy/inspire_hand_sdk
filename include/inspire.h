#ifndef INSPIRE_H
#define INSPIRE_H

#include <eigen3/Eigen/Dense>
#include <vector>

#include "SerialPort.h"

namespace inspire
{

class InspireHand
{
public:
  InspireHand(SerialPort::SharedPtr serial = nullptr, id_t id = 0)
  : serial_(serial), id(id)
  {
    if(!serial)
      serial_ = std::make_shared<SerialPort>("/dev/ttyUSB0", B115200);

  }

  void ChangeID(uint8_t before, uint8_t now)
  {
    std::vector<uint8_t> cmd = {0xEB, 0x90, before, 0x04, 0x12, 0xE8, 0x03, now, 0x00};
    cmd.back() = CheckSum(cmd.data(), cmd.size());
    serial_->send(cmd.data(), cmd.size());

    usleep(5000);
    serial_->recv(recvBuff, 9);
  }


  /**
   * @brief 设置各自由度的位置
   * -1: 不变
   * [0, 1] 0: 闭合
   * 小拇指、无名指、中指、食指、大拇指弯曲、大拇指旋转
   */
  int16_t SetPosition(const Eigen::Matrix<double, 6, 1> & q)
  {
    // 将q限制在0到1
    Eigen::Matrix<int16_t, 6, 1> q_int16 = (q * 1000).cast<int16_t>().cwiseMax(0).cwiseMin(1000);


    uint8_t cmd[20];
    cmd[0] = 0xEB;    // head
    cmd[1] = 0x90;
    cmd[2] = id;      // ID
    cmd[3] = 0x0F;    // data length
    cmd[4] = 0x12;    // write
    cmd[5] = 0xCE;    // address
    cmd[6] = 0x05;

    cmd[7] = q_int16(0) & 0xFF;
    cmd[8] = (q_int16(0) >> 8) & 0xFF;
    cmd[9] = q_int16(1) & 0xFF;
    cmd[10] = (q_int16(1) >> 8) & 0xFF;
    cmd[11] = q_int16(2) & 0xFF;
    cmd[12] = (q_int16(2) >> 8) & 0xFF;
    cmd[13] = q_int16(3) & 0xFF;
    cmd[14] = (q_int16(3) >> 8) & 0xFF;
    cmd[15] = q_int16(4) & 0xFF;
    cmd[16] = (q_int16(4) >> 8) & 0xFF;
    cmd[17] = q_int16(5) & 0xFF;
    cmd[18] = (q_int16(5) >> 8) & 0xFF;

    cmd[19] = CheckSum(cmd, 20);

    serial_->send(cmd, 20);
    usleep(5000);
    serial_->recv(recvBuff, 9);
    return 0;
  }

  /**
   * @brief 读取当前各自由度的角度位置
   * 
   * 小拇指、无名指、中指、食指、大拇指弯曲、大拇指旋转
   */
  int16_t GetPosition(Eigen::Matrix<double, 6, 1> & q)
  {
    std::vector<uint8_t> cmd = {0xEB, 0x90, id, 0x04, 0x11, 0x0A, 0x06, 0x0C, 0x00};
    cmd.back() = CheckSum(cmd.data(), cmd.size());
    serial_->send(cmd.data(), cmd.size());

    usleep(5000);
    size_t len = serial_->recv(recvBuff, 20);

    if(len != 20) return 1;
    if(recvBuff[19] != CheckSum(recvBuff, 20)) return 2;

    q(0) = (recvBuff[7] | (recvBuff[8] << 8)) / 1000.;
    q(1) = (recvBuff[9] | (recvBuff[10] << 8)) / 1000.;
    q(2) = (recvBuff[11] | (recvBuff[12] << 8)) / 1000.;
    q(3) = (recvBuff[13] | (recvBuff[14] << 8)) / 1000.;
    q(4) = (recvBuff[15] | (recvBuff[16] << 8)) / 1000.;
    q(5) = (recvBuff[17] | (recvBuff[18] << 8)) / 1000.;

    return 0;
  }

  /**
   * @brief 设置各自由度的速度
   * 
   * 设置完速度后会立即生效，断电再上电不保存。
   */
  void SetVelocity(int16_t v0, int16_t v1, int16_t v2, int16_t v3, int16_t v4, int16_t v5)
  {
    uint8_t cmd[20];
    cmd[0] = 0xEB;    // head
    cmd[1] = 0x90;
    cmd[2] = id;      // ID
    cmd[3] = 0x0F;    // data length
    cmd[4] = 0x12;    // write
    cmd[5] = 0xF2;    // address
    cmd[6] = 0x05;

    cmd[7] = v0 & 0xFF;
    cmd[8] = (v0 >> 8) & 0xFF;
    cmd[9] = v1 & 0xFF;
    cmd[10] = (v1 >> 8) & 0xFF;
    cmd[11] = v2 & 0xFF;
    cmd[12] = (v2 >> 8) & 0xFF;
    cmd[13] = v3 & 0xFF;
    cmd[14] = (v3 >> 8) & 0xFF;
    cmd[15] = v4 & 0xFF;
    cmd[16] = (v4 >> 8) & 0xFF;
    cmd[17] = v5 & 0xFF;
    cmd[18] = (v5 >> 8) & 0xFF;

    cmd[19] = CheckSum(cmd, 20);

    serial_->send(cmd, 20);
    usleep(5000);
    serial_->recv(recvBuff, 9);
  }

  /**
   * @brief 各自由度的力控阈值设置值
   * 
   * [0, 1000] Unit: g
   */
  void SetForce(uint16_t f0, uint16_t f1, uint16_t f2, uint16_t f3, uint16_t f4, uint16_t f5)
  {
    uint8_t cmd[20];
    cmd[0] = 0xEB;    // head
    cmd[1] = 0x90;
    cmd[2] = id;      // ID
    cmd[3] = 0x0F;    // data length
    cmd[4] = 0x12;    // write
    cmd[5] = 0xDA;    // address
    cmd[6] = 0x05;

    cmd[7] = f0 & 0xFF;
    cmd[8] = (f0 >> 8) & 0xFF;
    cmd[9] = f1 & 0xFF;
    cmd[10] = (f1 >> 8) & 0xFF;
    cmd[11] = f2 & 0xFF;
    cmd[12] = (f2 >> 8) & 0xFF;
    cmd[13] = f3 & 0xFF;
    cmd[14] = (f3 >> 8) & 0xFF;
    cmd[15] = f4 & 0xFF;
    cmd[16] = (f4 >> 8) & 0xFF;
    cmd[17] = f5 & 0xFF;
    cmd[18] = (f5 >> 8) & 0xFF;

    cmd[19] = CheckSum(cmd, 20);

    serial_->send(cmd, 20);
    usleep(5000);
    serial_->recv(recvBuff, 9);
  }

  /**
   * @brief 各手指的实际受力
   * 
   * 原本单位为 g, [0 - 1000], 转为 N
   */
  int16_t GetForce(Eigen::Matrix<double, 6, 1> & f)
  {
    std::vector<uint8_t> cmd = {0xEB, 0x90, id, 0x04, 0x11, 0x2E, 0x06, 0x0C, 0x00};
    cmd.back() = CheckSum(cmd.data(), cmd.size());
    serial_->send(cmd.data(), cmd.size());

    usleep(5000);
    size_t len = serial_->recv(recvBuff, 20);

    if(len != 20) return 1;
    if(recvBuff[19] != CheckSum(recvBuff, 20)) return 2;

    f(0) = int16_t(recvBuff[7] | (recvBuff[8] << 8)) / 1000. * 9.8;
    f(1) = int16_t(recvBuff[9] | (recvBuff[10] << 8)) / 1000. * 9.8;
    f(2) = int16_t(recvBuff[11] | (recvBuff[12] << 8)) / 1000. * 9.8;
    f(3) = int16_t(recvBuff[13] | (recvBuff[14] << 8)) / 1000. * 9.8;
    f(4) = int16_t(recvBuff[15] | (recvBuff[16] << 8)) / 1000. * 9.8;
    f(5) = int16_t(recvBuff[17] | (recvBuff[18] << 8)) / 1000. * 9.8;

    return 0;
  }

  /**
   * @brief 清除错误
   * 
   * 当灵巧手发生堵转，过流，异常等故障可通过清除故障指令恢复正常运行
   */
  void ClearError()
  {
    uint8_t cmd[9];
    cmd[0] = 0xEB;    // head
    cmd[1] = 0x90;
    cmd[2] = id;      // ID
    cmd[3] = 0x04;    // data length
    cmd[4] = 0x12;    // write
    cmd[5] = 0xEC;    // address
    cmd[6] = 0x03;
    cmd[7] = 0x01;
    cmd[8] = CheckSum(cmd, 9);
    
    serial_->send(cmd, 9);
    usleep(5000);
    serial_->recv(recvBuff, 9);
  }

  /**
   * @brief 受力传感器校准
   * 
   * 校准时需保证灵巧手处于空载状态
   */
  void Calibration()
  {
    std::vector<uint8_t> cmd = {0xEB, 0x90, id, 0x04, 0x12, 0x2F, 0x06, 0x01, 0x00};
    cmd.back() = CheckSum(cmd.data(), cmd.size());
    serial_->send(cmd.data(), cmd.size());

    usleep(5000);
    serial_->recv(recvBuff, 9); // 第一帧
    sleep(10); // 整个过程大约需要6s
    serial_->recv(recvBuff, 9); // 第二帧
    std::cout << "Calibration: ";
    print_data(recvBuff, 9);
  }

  uint8_t id = 1;
private:
  uint8_t CheckSum(const uint8_t* data, uint8_t len)
  {
    uint8_t sum = 0;
    // 除应答帧头外其余数据的累加和的低字节
    for (int i = 2; i < len - 1; i++)
    {
      sum += data[i];
    }
    return sum;
  }

  SerialPort::SharedPtr serial_;
  uint8_t recvBuff[1024];
};

} // namespace inspire
#endif // INSPIRE_H