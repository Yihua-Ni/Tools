#ifndef PCAP_LIDAR
#define PCAP_LIDAR

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <pcap/pcap.h>
#include <Eigen/Core>

#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

namespace rsdata 
{
  static int VERT_ANGLE[128];
  static int HORI_ANGLE[128];
  static int g_ChannelNum[128][51];

  static float temper = 31.0;
  static int tempPacketNum = 0;
  static int numOfLasers = 128;
  static int TEMPERATURE_RANGE = 40;
  static float lastAzimuth = -1;
  static float azimuthDiff = 10;


  static const int TEMPERATURE_MIN = 31;
  static const int BLOCKS_PER_PACKET_RS128 = 3;

  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 128;
  static const int BLOCK_DATA_SIZE_RS128 = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 384
  static const int PACKET_STATUS_SIZE = 4;

  static const float RS128_DSR_TOFFSET = 3.23f;       // [µs]
  static const float RS128_BLOCK_TDURATION = 55.55f;  // [µs]

  static const float DISTANCE_RESOLUTION = 0.005f;      /**< meters */

  union two_bytes
  {
    uint16_t uint;
    uint8_t bytes[2];
  };

  struct Packet {
      uint64_t timestamp;
      uint8_t data[1248];
  };

  typedef struct raw_block_rs128
  {
    uint8_t header;
    uint8_t retWaveId;
    uint8_t rotation_1;
    uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE_RS128];  // 384
  } raw_block_rs128;

  typedef struct raw_packet_rs128
  {
    raw_block_rs128 blocks[BLOCKS_PER_PACKET_RS128];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE];
  } raw_packet_rs128;

  class RawData
  {
    public:
      RawData();
      int unpack_rs128(const Packet& pkt, std::vector<Eigen::Vector3f>& p3ds, double& ts);
      void init();

      bool is_init_angle_;
    private:

      int estimateTemperature(float Temper);
      unsigned int correctAzimuth(float azimuth_f, int passageway);
      float pixelToDistance(int pixel_value, int passageway);
      float computeTemperature128(unsigned char bit2, unsigned char bit1);
      float Rx_;  // the optical center position in the lidar coordination in x direction
      float Ry_;  // the optical center position in the lidar coordination in y direction, for now not used
      float Rz_;  // the optical center position in the lidar coordination in z direction
      bool angle_flag_;
      float start_angle_;
      float end_angle_;
      float max_distance_;
      float min_distance_;
      int return_mode_;
      bool info_print_flag_;
      int block_num;

      /* cos/sin lookup table */
      std::vector<double> cos_lookup_table_;
      std::vector<double> sin_lookup_table_;
  };

}

namespace rsdriver
{
  static uint16_t MSOP_DATA_PORT_NUMBER = 6699;   // rslidar default data port on PC
  static uint16_t DIFOP_DATA_PORT_NUMBER = 7788;  // rslidar default difop data port on PC
  static const size_t packet_size = sizeof(rsdata::Packet().data);

  class Input
  {
  public:
    Input(uint16_t port);

    virtual ~Input()
    {
    }

    virtual int getPacket(rsdata::Packet* pkt, const double time_offset) = 0;

    int getRpm(void);
    int getReturnMode(void);
    bool getUpdateFlag(void);
    void clearUpdateFlag(void);
    virtual void setNPackets(int npackets);

  protected:
    uint16_t port_;
    std::string devip_str_;
    int cur_rpm_;
    int return_mode_;
    bool npkt_update_flag_;
  };

  class InputPCAP : public Input
  {
  public:
    InputPCAP(uint16_t port = MSOP_DATA_PORT_NUMBER,
              std::string filename = "", bool read_once = false, double repeat_delay = 0.0);

    virtual ~InputPCAP();

    virtual int getPacket(rsdata::Packet* pkt, const double time_offset);
    virtual void setNPackets(int npackets);

  private:
    std::string filename_;
    pcap_t* pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    int read_hz_;
    unsigned int read_sleep_us_;
    double repeat_delay_;
  };
}

#endif