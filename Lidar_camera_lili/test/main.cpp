#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include<Eigen/Cholesky>
#include "pcap/pcap_lidar.h"
#include "viewer/viewer.h"
#include "tools/tools.h"
#include "tools/post.h"
#include "dbscan/dbscan.h"

int main(int argc, char const *argv[])
{
    bool do_viz = true;
    bool have_mask = false;
    bool do_proj_lidar_on_img = true;

    // rsdriver::InputPCAP pcap(1234, "/home/lili/D/0727_vio/lidar/3/2020-07-27-11-04-50-RS-128-Data.pcap");
    // rsdriver::InputPCAP pcap(1234, "/home/lili/D/0727_vio/lidar/3/2020-07-27-11-12-46-RS-128-Data.pcap");
    rsdriver::InputPCAP pcap(1234, "/home/minieye/Lidar_camera/data/2020-07-27-11-21-17-RS-128-Data.pcap");
    rsdata::Packet pkt;
    rsdata::RawData rData;
    PostProcess post;

    // std::string videoFolder = "/home/lili/D/0727_vio/cam/No_3_lane/20200727111250/converted_data";
    std::string videoFolder = "/home/minieye/Lidar_camera/data/converted_data";
    std::string logPath = videoFolder + "/log.txt";
    std::vector<std::pair<double, int>> frameInfo = post.readVideoLog(logPath);

    std::string posePath = videoFolder + "/pose.txt";
    post.readPoses(posePath);

    std::string videoPath = videoFolder + "/output.mp4";
    cv::VideoCapture cap(videoPath);
    cv::Mat K = (cv::Mat_<float>(3,3) << 1468.2, 0, 650.7666, 0, 1479.3, 332.4450, 0, 0, 1);
    cv::Mat D = (cv::Mat_<float>(1,5) << -0.4346, -0.0605, 0.0044, 0.0033, 0.5326);
    post.setCamera((float*)K.data, (float*)D.data);

    Eigen::Vector3f r_cl(1.237030913495427, -1.218578471367221, 1.206726188175037);
    Eigen::Vector3f t_l_c(-2.266658751294206, -679.8422553732235, -731.4916429990781);
    t_l_c *= 1e-3;
    post.setExtLidar2Cam(r_cl, t_l_c);

    if (!cap.isOpened())
    {
        std::cerr << "can't read video : " << videoPath << std::endl;
        return -1;
    }
    

    Viewer viewer;

    if (do_viz)
    {
        viewer.init();
    }

    double dur = 0.1;     //100毫秒
    bool bSync = false;
    double lidar_ts = 0.0;

    std::deque<std::pair<double, std::vector<Eigen::Vector3f>>> pts2com;

    for (int i = 0; i < frameInfo.size(); i++)
    {
        double curr_ts = frameInfo[i].first;
        double curr_id = frameInfo[i].second;

        if (!bSync)
        {
            while (lidar_ts < curr_ts - dur)
            {
                bool bOK = pcap.getPacket(&pkt, 0);
                if(!bOK)
                {
                    std::cerr << "bad pcap package." << std::endl;
                    continue;
                }
                std::vector<Eigen::Vector3f> p3ds;
                rData.unpack_rs128(pkt, p3ds, lidar_ts);
            }

            if (lidar_ts > curr_ts + dur)
            {
                continue;
            }

            bSync = true;
        }

        while (pcap.getPacket(&pkt, 0) > 0)
        {
            std::vector<Eigen::Vector3f> p3ds;
            rData.unpack_rs128(pkt, p3ds, lidar_ts);

            pts2com.emplace_back(lidar_ts, p3ds);
            if(lidar_ts > curr_ts + dur)
            {
                break;
            }
        }

        for(auto ite_pts2com = pts2com.begin(); ite_pts2com != pts2com.end();)
        {
            if (ite_pts2com->first < curr_ts - dur)
            {
                ite_pts2com = pts2com.erase(ite_pts2com);
            }
            else
            {
                break;
            }
        }

        std::vector<Eigen::Vector3f> local_pts;
        if (!pts2com.empty())
        {
            for(auto& it : pts2com)
            {
                std::vector<Eigen::Vector3f> res = post.cvt2CamCoord(it.second, it.first, curr_id);
                local_pts.insert(local_pts.end(), res.begin(), res.end());
            }

            if (do_viz)
            {
                viewer.insertPoints(curr_ts, local_pts, "lidar");
            }
        }


        if(do_viz)
        {
            while (cap.get(CV_CAP_PROP_POS_FRAMES) <= curr_id) 
            {
                cap.grab();
            }
            cv::Mat im;
            cap.retrieve(im);

            if (do_proj_lidar_on_img)
            {

                if (!viewer.bShowIpm_)
                {
                    std::vector<Eigen::Vector2f> uvs = post.projPts2Img(local_pts);
                    for(auto& uv : uvs)
                    {
                        cv::circle(im, cv::Point2f(uv(0), uv(1)), 1, cv::Scalar(0,0,255), 1);
                    }
                    viewer.insertImg(im);
                }
                else
                {
                    cv::Mat ipm(im.size(), CV_8UC3, cv::Scalar(0, 0, 0));

                    std::vector<Eigen::Vector2f> ipmUvs1 = post.drawIPMuvs(local_pts, 150, 100);
                    for(auto& uv : ipmUvs1)
                    {
                        cv::circle(ipm, cv::Point2f(uv(0), uv(1)), 1, cv::Scalar(0,255,0), 1);
                    }

                    viewer.insertImg(ipm);
                }
            }
            else
            {
                viewer.insertImg(im);
            }

            if(viewer.isStop())
            {
                break;
            }
        }
    }



    return 0;
}
