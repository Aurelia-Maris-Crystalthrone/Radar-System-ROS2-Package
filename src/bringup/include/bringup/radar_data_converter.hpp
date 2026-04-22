#ifndef RADAR_DATA_CONVERTER_HPP_
#define RADAR_DATA_CONVERTER_HPP_

#include <cstddef>    // for size_t
#include <cstdint>    // for uint8_t, uint16_t
#include <vector>

namespace bringup
{

struct RadarData
{
  double distance;
  float position_x;
  float position_y;
  float position_z;
  float base_point_x;
  float base_point_y;
  float base_point_z;
};

class RadarDataProtocol
{
public:
  static constexpr uint8_t FRAME_HEADER = 0xAA;
  static constexpr uint8_t FRAME_FOOTER = 0x55;
  static constexpr size_t MAX_PAYLOAD_SIZE = 64;

  static std::vector<uint8_t> packRadarData(const RadarData & data)
  {
    std::vector<uint8_t> payload;

    // 1. 打包距离 (double, 8 字节)
    const uint8_t * dist_bytes = reinterpret_cast<const uint8_t *>(&data.distance);
    payload.insert(payload.end(), dist_bytes, dist_bytes + sizeof(double));

    // 2. 打包雷达位置 (3 个 float, 12 字节)
    const uint8_t * pos_bytes = reinterpret_cast<const uint8_t *>(&data.position_x);
    payload.insert(payload.end(), pos_bytes, pos_bytes + sizeof(float));
    pos_bytes = reinterpret_cast<const uint8_t *>(&data.position_y);
    payload.insert(payload.end(), pos_bytes, pos_bytes + sizeof(float));
    pos_bytes = reinterpret_cast<const uint8_t *>(&data.position_z);
    payload.insert(payload.end(), pos_bytes, pos_bytes + sizeof(float));

    // 3. 打包基地位置 (3 个 float, 12 字节)
    const uint8_t * base_bytes = reinterpret_cast<const uint8_t *>(&data.base_point_x);
    payload.insert(payload.end(), base_bytes, base_bytes + sizeof(float));
    base_bytes = reinterpret_cast<const uint8_t *>(&data.base_point_y);
    payload.insert(payload.end(), base_bytes, base_bytes + sizeof(float));
    base_bytes = reinterpret_cast<const uint8_t *>(&data.base_point_z);
    payload.insert(payload.end(), base_bytes, base_bytes + sizeof(float));

    // 4. 计算并打包差值 (雷达位置 - 基地位置, 3 个 float, 12 字节)
    float dx = data.position_x - data.base_point_x;
    float dy = data.position_y - data.base_point_y;
    float dz = data.position_z - data.base_point_z;

    const uint8_t * dx_bytes = reinterpret_cast<const uint8_t *>(&dx);
    payload.insert(payload.end(), dx_bytes, dx_bytes + sizeof(float));
    const uint8_t * dy_bytes = reinterpret_cast<const uint8_t *>(&dy);
    payload.insert(payload.end(), dy_bytes, dy_bytes + sizeof(float));
    const uint8_t * dz_bytes = reinterpret_cast<const uint8_t *>(&dz);
    payload.insert(payload.end(), dz_bytes, dz_bytes + sizeof(float));

    // 5. 构建完整帧
    std::vector<uint8_t> frame;
    frame.push_back(FRAME_HEADER);

    uint16_t payload_len = static_cast<uint16_t>(payload.size());
    frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));

    frame.insert(frame.end(), payload.begin(), payload.end());

    uint8_t checksum = 0;
    for (const auto & byte : payload) {
      checksum ^= byte;
    }
    frame.push_back(checksum);
    frame.push_back(FRAME_FOOTER);

    return frame;
  }
};

}  // namespace bringup

#endif  // RADAR_DATA_CONVERTER_HPP_