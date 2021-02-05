#include "pcap_lidar.h"
#include <sstream>
#include <math.h>

namespace rsdata
{
    int RawData::unpack_rs128(const Packet& pkt, std::vector<Eigen::Vector3f>& p3ds, double& ts)
    {
        if((pkt.data[0] & 0xff) != 0x55 && (pkt.data[1] & 0xff) != 0xaa)
        {
            return -1;
        }

        float curAzimuth;  // 0.01 dgree
        float intensity;
        float azimuth_corrected_f;
        unsigned int azimuth_corrected;

        const raw_packet_rs128* raw = (const raw_packet_rs128*)&pkt.data[80];

        //time
        uint8_t time_raw_s[6]; //s
        memcpy(time_raw_s, &(pkt.data[10]), sizeof(time_raw_s));
        uint64_t time_s = time_raw_s[0]*pow(256,5) + time_raw_s[1]*pow(256,4) + time_raw_s[2]*pow(256,3) + time_raw_s[3]*pow(256,2) + time_raw_s[4]*pow(256,1) + time_raw_s[5];

        uint8_t time_raw_ns[4]; //ns
        memcpy(time_raw_ns, &(pkt.data[16]), sizeof(time_raw_ns));
        uint64_t time_ns = time_raw_ns[0]*pow(256,3) + time_raw_ns[1]*pow(256,2) + time_raw_ns[2]*pow(256,1) + time_raw_ns[3];
  
        double time_s_ns = time_s + time_ns/pow(10, 9);
        ts = time_s_ns + 0.97;
        // uint64_t time_llt = uint64_t(time_s * 1e6 + time_ns * 1e-3);

        if (tempPacketNum < 600 && tempPacketNum > 0)
        {
          tempPacketNum++;
        }
        else
        {
          temper = computeTemperature128(pkt.data[8], pkt.data[9]);
          // ROS_INFO_STREAM("Temp is: " << temper);
          tempPacketNum = 1;
        }




        for (int block = 0; block < BLOCKS_PER_PACKET_RS128; block++, this->block_num++)  // 1 packet:3 data blocks
        {
          // calculte current azimuth
          curAzimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

          // get the real azimuth diff
          if ((lastAzimuth != -1) && (lastAzimuth != curAzimuth))
          {
            azimuthDiff = (float)((int)(36000 + curAzimuth - lastAzimuth) % 36000);
          }

          // Ingnore the block if the azimuth change abnormal
          if (azimuthDiff <= 0.0 || azimuthDiff > 40)
          {
            continue;
          }

          // process firing data
          for (int dsr = 0; dsr < 128; dsr++)
          {
            //dsr_temp
            int dsr_temp;
            dsr_temp = dsr / 4;
            dsr_temp = dsr_temp % 16;

            // azimuth
            azimuth_corrected_f = curAzimuth + (azimuthDiff * (dsr_temp * RS128_DSR_TOFFSET) / RS128_BLOCK_TDURATION);
            azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

            // distance
            union two_bytes tmp;
            tmp.bytes[1] = raw->blocks[block].data[dsr * 3];
            tmp.bytes[0] = raw->blocks[block].data[dsr * 3 + 1];
            int distance = tmp.uint;

            // calculate intensity
            intensity = (float)raw->blocks[block].data[dsr * 3 + 2];

            // calculate distance
            float distance2 = pixelToDistance(distance, dsr);
            distance2 = distance2 * DISTANCE_RESOLUTION;

            unsigned int arg_horiz_orginal = (unsigned int)(round(azimuth_corrected_f)) % 36000;
            unsigned int arg_horiz = azimuth_corrected;
            unsigned int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;
            // pcl::PointXYZI point;

            if (distance2 > max_distance_ || distance2 < min_distance_ ||
                (angle_flag_ && (arg_horiz < start_angle_ || arg_horiz > end_angle_)) ||
                (!angle_flag_ && (arg_horiz > end_angle_ && arg_horiz < start_angle_)))  // invalid distance
            {
              // point.x = NAN;
              // point.y = NAN;
              // point.z = NAN;
              // point.intensity = 0.0f;
              // pointcloud->push_back(point);
              // pointcloud->at(this->block_num, dsr) = point;
            }
            else
            {
              // If you want to fix the rslidar X aixs to the front side of the cable, please use below
              float x = distance2 * this->cos_lookup_table_[arg_vert] * this->cos_lookup_table_[arg_horiz] + 
                        Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
              float y = -distance2 * this->cos_lookup_table_[arg_vert] * this->sin_lookup_table_[arg_horiz] - 
                        Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
              float z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;

            //   point.intensity = intensity;

              // if (intensity < 10.0||z > 0)
              if (intensity < 10.0)
              {
                continue;
              }

              p3ds.emplace_back(-y,x,z);

            //   pointcloud->at(this->block_num, dsr) = point;
              // std::cout << this->block_num << ", " << dsr << std::endl;
              // pointcloud->push_back(point);
            }
          }
          // after process firing data, update lastAzimuth
          lastAzimuth = curAzimuth;
        }


        return 1;
    }

    int RawData::estimateTemperature(float Temper)
    {
      int temp = int(Temper + 0.5);
      if (temp < TEMPERATURE_MIN)
      {
        temp = TEMPERATURE_MIN;
      }
      else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE)
      {
        temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
      }

      return temp;
    }

    float RawData::pixelToDistance(int pixel_value, int passageway)
    {
      int input_temp_index = estimateTemperature(temper) - TEMPERATURE_MIN;
      float delta_dist = g_ChannelNum[passageway][input_temp_index];

      float esti_distance = 0.0f;
      if (pixel_value < delta_dist)
      {
        esti_distance = 0.0f;
      }
      else
      {
        esti_distance = (float)(pixel_value - delta_dist);
      }
      return esti_distance;
    }

    unsigned int RawData::correctAzimuth(float azimuth_f, int passageway)
    {
      unsigned int azimuth;
      if (azimuth_f > 0.0f && azimuth_f < 3000.0f)
      {
        azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
      }
      else
      {
        azimuth_f = azimuth_f + HORI_ANGLE[passageway];
      }

      if (azimuth_f < 0)
      {
        azimuth_f += 36000.0f;
      }
      azimuth = (unsigned int)azimuth_f;
      azimuth %= 36000;

      return azimuth;
    }

    float RawData::computeTemperature128(unsigned char bit2, unsigned char bit1)
    {
      float Temp;
      float bitneg = bit2 & 128;   // 10000000
      float highbit = bit2 & 127;  // 01111111
      float lowbit = bit1 >> 4;
      if (bitneg == 128)
      {
        Temp = -1.0f * (highbit * 16 + lowbit) * 0.0625f;
      }
      else
      {
        Temp = (highbit * 16.0f + lowbit) * 0.0625f;
      }

      return Temp;
    }

    void RawData::init()
    {

        std::string model = "RS128";
        std::string anglePath = "../data/rs_lidar_128/angle.csv";
        std::string channelPath = "../data/rs_lidar_128/ChannelNum.csv";
        std::string input_difop_packets_topic;

        this->is_init_angle_ = false;
        this->block_num = 0;

        // lookup table init
        this->cos_lookup_table_.resize(36000);
        this->sin_lookup_table_.resize(36000);
        for (unsigned int i = 0; i < 36000; i++)
        {
          double rad = RS_TO_RADS(i / 100.0f);

          this->cos_lookup_table_[i] = std::cos(rad);
          this->sin_lookup_table_[i] = std::sin(rad);
        }

        start_angle_ = 0.0;
        end_angle_ = 360.0;

        if (start_angle_ < 0.0f || start_angle_ > 360.0f || end_angle_ < 0.0f || end_angle_ > 360.0f)
        {
          start_angle_ = 0.0f;
          end_angle_ = 360.0f;
          std::cout << "start angle and end angle select feature deactivated." << std::endl;
        }
        else
        {
          std::cout << "start angle and end angle select feature activated." << std::endl;
        }

        angle_flag_ = true;
        if (start_angle_ > end_angle_)
        {
          angle_flag_ = false;
          std::cout << "Start angle is smaller than end angle, not the normal state!" << std::endl;
        }

        std::cout << "start_angle: " << start_angle_ << " end_angle: " << end_angle_ << " angle_flag: " << angle_flag_ << std::endl;

        start_angle_ = start_angle_ * 100;
        end_angle_ = end_angle_ * 100;

        max_distance_ = 200.0;
        min_distance_ = 0.2;

        std::cout  << "distance threshlod, max: " << max_distance_ << ", min: " << min_distance_ << std::endl;
    
        info_print_flag_ = false;

        if (model == "RS128")
        {
          numOfLasers = 128;
          TEMPERATURE_RANGE = 50;
          Rx_ = 0.03615f;
          Ry_ = -0.017f;
          Rz_ = 0.0f;
        }
        else
        {
          std::cout << "Bad model!" << std::endl;
        }

        // return mode default
        return_mode_ = 1;

        FILE* f_angle = fopen(anglePath.c_str(), "r");
        if (!f_angle)
        {
          std::cerr << anglePath << " does not exist" << std::endl;
        }
        else
        {
          float b[128], d[128];
          int loopk = 0;
          int loopn = 0;
          while (!feof(f_angle))
          {
            int tmp = fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
            loopk++;
            if (loopk > (numOfLasers - 1))
              break;
          }
          for (loopn = 0; loopn < numOfLasers; loopn++)
          {
            VERT_ANGLE[loopn] = (int)(b[loopn] * 100);
            HORI_ANGLE[loopn] = (int)(d[loopn] * 100);
            if (model == "RS16")
            {
              HORI_ANGLE[loopn] = 0.0f;
            }
          }
          fclose(f_angle);
        }

        FILE* f_channel = fopen(channelPath.c_str(), "r");
        if (!f_channel)
        {
          std::cerr << channelPath << " does not exist" << std::endl;
        }
        else
        {
          int loopl = 0;
          int loopm = 0;
          int c[51];
          int tempMode = 1;
          while (!feof(f_channel))
          { 
            int tmp = fscanf(f_channel,
                              "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                              "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                              &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                              &c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
                              &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
                              &c[37], &c[38], &c[39], &c[40], &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48],
                              &c[49], &c[50]);

            for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++)
            {
              g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
            }
            loopm++;
            if (loopm > (numOfLasers - 1))
            {
              break;
            }
          }
          fclose(f_channel);
        }

        return;
    }

    RawData::RawData()
    {
        init();
        return;
    }
}

namespace rsdriver
{
    Input::Input(uint16_t port) : port_(port)
    {
      npkt_update_flag_ = false;
      cur_rpm_ = 600;
      return_mode_ = 1;
    }

    int Input::getRpm(void)
    {
      return cur_rpm_;
    }

    int Input::getReturnMode(void)
    {
      return return_mode_;
    }

    bool Input::getUpdateFlag(void)
    {
      return npkt_update_flag_;
    }

    void Input::clearUpdateFlag(void)
    {
      npkt_update_flag_ = false;
    }

    void Input::setNPackets(int npackets)
    {
    }

    InputPCAP::InputPCAP(uint16_t port, std::string filename, bool read_once,
                         double repeat_delay)
      : Input(port), filename_(filename)
    {
        pcap_ = NULL;
        empty_ = true;

        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
        {
          std::cerr << "Error opening rslidar socket dump file." << std::endl;
          return;
        }
        

        std::stringstream filter;
        if (devip_str_ != "")  // using specific IP?
        {
          filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        int ret = pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);

        if (ret < 0) {
            std::cerr << "[driver][pcap] pcap compile filter fail. filter: "
                        << filter.str() << std::endl;
            return;
        }
    }


    void InputPCAP::setNPackets(int npackets)
    {
    }

    /** destructor */
    InputPCAP::~InputPCAP(void)
    {
      pcap_close(pcap_);
    }


    int InputPCAP::getPacket(rsdata::Packet* pkt, const double time_offset)
    {
        clearUpdateFlag();

        struct pcap_pkthdr* header;
        const u_char* pkt_data;

        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
        {
            if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
            {
              return -1;
            }

            memcpy(&pkt->data[0], pkt_data + 42, packet_size);

            if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
            {  // difop
              int rpm = (pkt->data[8] << 8) | pkt->data[9];
              int mode = 1;

              if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08) ||
                  (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
              {
                if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
                {
                  mode = 0;
                }
              }

              if (cur_rpm_ != rpm || return_mode_ != mode)
              {
                cur_rpm_ = rpm;
                return_mode_ = mode;

                npkt_update_flag_ = true;
              }
            }
        }
        else
        {
            return -1;
        }

        return 1;
    }

};