#include "post.h"
#include <iostream>

PostProcess::PostProcess()
{
    fx = 1;
    fy = 1;
    cx = 0;
    cy = 0;
    k1 = 0;
    k2 = 0;
    p1 = 0;
    p2 = 0;
    k3 = 0;

    x_scale_ = 1.0;
    y_scale_ = 1.0;
    return;
};

void PostProcess::setCamera(float* K, float* D)
{
    fx = K[0];
    cx = K[2];
    fy = K[4];
    cy = K[5];

    k1 = D[0];
    k2 = D[1];
    p1 = D[2];
    p2 = D[3];
    k3 = D[4];

    return;
}


std::vector<Eigen::Vector3f> PostProcess::cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts, const double ts, const int id, bool do_filter, bool do_dist)
{
    std::vector<Eigen::Vector3f> res;
    double curr_ts = frm_map_[id]->ts;
    int lid = -1;
    int rid = -1;
    if (ts < curr_ts)
    {
        for (int i = id; i >= 0; i--)
        {
            if (!frm_map_.count(i))
            {
                continue;
            }

            double lt = frm_map_[i]->ts;
            if (lt < ts)
            {
                lid = i;
                rid = i + 1;
                break;
            }
        }
    }
    else
    {
        for (int i = id; i < frms_.back().id; i++)
        {
            if (!frm_map_.count(i))
            {
                continue;
            }

            double rt = frm_map_[i]->ts;
            if (rt > ts)
            {
                rid = i;
                lid = i - 1;
                break;
            }
        }
    }

    if (frm_map_.count(lid) && frm_map_.count(rid) && frm_map_.count(id))
    {
        auto& frm_l = frm_map_[lid];
        auto& frm_r = frm_map_[rid];
        auto& frm = frm_map_[id];

        double r = (ts - frm_l->ts) / (frm_r->ts - frm_l->ts);

        const Sophus::SE3f& Twc0 = frm_l->Twc;
        const Sophus::SE3f& Twc1 = frm_r->Twc;
        const Sophus::SE3f& Twc = Twc0 * Sophus::SE3f::exp(r * (Twc0.inverse() * Twc1).log());
        const Sophus::SE3f& Tcl = frm->Twc.inverse() * Twc * Tcl_;

        Sophus::SE3f T_fil = Twc0.inverse() * Twc * Tcl_;
        int id_fil = lid;
        if (r > 0.5)
        {
            T_fil = Twc1.inverse() * Twc * Tcl_;
            id_fil = rid;
        }
        
        bool bMask = false;
        cv::Mat* im = nullptr;
        if (mask_map_.count(id_fil))
        {
            im = &mask_map_[id_fil];
            bMask = true;
        }
        // cv::Mat test(im->size(), CV_8UC1, cv::Scalar(0));

        for(const auto& it : pts)
        {

            if (bMask)
            {
                Eigen::Vector3f pc = T_fil * it;
                if (0 > pc(2))
                {
                    continue;
                }
                
                Eigen::Vector3f uv = pc / pc(2);
                float x = uv(0);
                float y = uv(1);

                if(do_dist)
                {
                    float xy = x * y;
                    float x2 = x*x;
                    float y2 = y*y;
                    float r2 = x2 + y2;
                    float r4 = r2 * r2;
                    float r6 = r2 * r4;

                    float c0 = 1 + k1 * r2 + k2 * r4 + k3 * r6;
                    x = c0 * x + 2 * p1 * xy + p2 * (r2 + 2 * x2);
                    y = c0 * y + 2 * p2 * xy + p1 * (r2 + 2 * y2);
                }
                cv::Point2f pt;
                pt.x = (x * fx + cx) * x_scale_;
                pt.y = (y * fy + cy) * y_scale_;

                if(pt.x > 0 && pt.x < im->cols && pt.y > 0 && pt.y < im->rows)
                {
                    int val = im->at<uchar>(pt);
                    // if (val != 1 && val != 6)
                    if (val != 6)
                    {
                        continue;
                    }
                }
                else
                {
                    continue;
                }
                

                // res.emplace_back(Tcl * it + Eigen::Vector3f(1.0, 0,0));
                res.emplace_back(Tcl * it);
            }
            else
            {
                res.emplace_back(Tcl * it);
            }
        }
            // cv::imshow("test", test);
            // cv::waitKey(1);
    }

    return res;
}

std::vector<Eigen::Vector3f> PostProcess::cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts, const int id0, const int id1, const float r)
{
    std::vector<Eigen::Vector3f> res;
    const Sophus::SE3f& Twc0 = frm_map_[id0]->Twc;
    const Sophus::SE3f& Twc1 = frm_map_[id1]->Twc;
    const Sophus::SE3f& Twc = Twc0 * Sophus::SE3f::exp(r * (Twc0.inverse() * Twc1).log());
    const Sophus::SE3f& Tc1l = Twc1.inverse() * Twc * Tcl_;

    for(const auto& it : pts)
    {
        res.emplace_back(Tc1l * it);
    }

    return res;
}

std::vector<Eigen::Vector3f> PostProcess::cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts)
{
    std::vector<Eigen::Vector3f> res;

    for(const auto& it : pts)
    {
        res.emplace_back(Tcl_ * it);
    }

    return res;
}

void PostProcess::setExtLidar2Cam(const Eigen::Vector3f& r, const Eigen::Vector3f& t)
{
    Tcl_ = Sophus::SE3f(Sophus::SO3f::exp(r), t);
    return;
}

std::vector<Eigen::Vector2f> PostProcess::projPts2Img(const std::vector<Eigen::Vector3f>& pts, bool do_dist)
{
    std::vector<Eigen::Vector2f> uvs;

    for(const auto& it : pts)
    {
        if (0 > it(2))
        {
            continue;
        }
        
        Eigen::Vector3f uv = it / it(2);
        float x = uv(0);
        float y = uv(1);

        if(do_dist)
        {
            float xy = x * y;
            float x2 = x*x;
            float y2 = y*y;
            float r2 = x2 + y2;
            float r4 = r2 * r2;
            float r6 = r2 * r4;

            float c0 = 1 + k1 * r2 + k2 * r4 + k3 * r6;
            x = c0 * x + 2 * p1 * xy + p2 * (r2 + 2 * x2);
            y = c0 * y + 2 * p2 * xy + p1 * (r2 + 2 * y2);
        }

        uvs.emplace_back(x * fx + cx, y * fy + cy);
    }

    return uvs;
}

bool PostProcess::readPoses(const std::string& path)
{

    std::deque<std::pair<double, int>> vec;
	std::ifstream infile;
	infile.open(path.c_str());

    if (infile.is_open())
    {
        int cnt = 0;
	    std::string str;
	    while (std::getline(infile, str)) 
        {
            std::vector<std::string> strVec;
            simpleSplit(str, strVec, ',');

            if (8 != strVec.size())
            {
                std::cerr << " bad poses : " << str << std::endl;
                continue;
            }

            int id = atoi(strVec[0].c_str());
            double ts = atof(strVec[1].c_str());
            Eigen::Vector3f omega, twc;
            omega << atof(strVec[2].c_str()), atof(strVec[3].c_str()), atof(strVec[4].c_str()); 
            twc << atof(strVec[5].c_str()), atof(strVec[6].c_str()), atof(strVec[7].c_str()); 
            Sophus::SE3f Twc(Sophus::SO3f::exp(omega), twc);

            while (id != frms_[cnt].id)
            {
                cnt++;
            }

            frms_[cnt].Twc = Twc;
            frms_[cnt].isPosed = true;
        }
    }
    else
    {
        for(auto& it : frms_)
        {
            it.Twc = Sophus::SE3f(Sophus::SO3f::exp(Eigen::Vector3f(0,0,0)), Eigen::Vector3f(0,0,0));
            it.isPosed = true;
        }
    }
    
    


    // interpolation
    {
        bool bInit = false;
        Frame *prev_frm = nullptr;
        std::vector<Frame*> frm2interp;

        for (int i = 0; i < frms_.size(); i++)
        {
            Frame& frm = frms_[i];
            if (!frm.isPosed)
            {
                frm2interp.push_back(&frm);
                continue;
            }

            if (!bInit)
            {
                for(auto& it : frm2interp)
                {
                    it->Twc = frm.Twc;
                    it->isPosed = true;
                }
                bInit = true;
            }
            else
            {
                double t0 = prev_frm->ts;
                Sophus::SE3f Twc0 = prev_frm->Twc;
                
                double t1 = frm.ts;
                Sophus::SE3f Twc1 = frm.Twc;

                for(auto& it : frm2interp)
                {
                    float r = (it->ts - t0) / (t1 - t0);
                    Sophus::SE3f Twc = Twc0 * Sophus::SE3f::exp(r * (Twc0.inverse() * Twc1).log());
                    it->Twc = Twc;
                    it->isPosed = true;
                }
            }
            frm2interp.clear();
            prev_frm = &frm;
        }

        for(auto& it : frm2interp)
        {
            it->Twc = prev_frm->Twc;
            it->isPosed = true;
        }
    }

    for (int i = 0; i < frms_.size(); i++)
    {
        if (frms_[i].isPosed)
        {
            frm_map_[frms_[i].id] = &frms_[i];
        }
    }

    return true;
}

Frame* PostProcess::getPosedFrm(int id)
{
    if (frm_map_.count(id))
    {
        return frm_map_[id];
    }
    return nullptr;
}

std::unordered_map<int, std::vector<Eigen::Vector3f>> PostProcess::readLanePts(const std::string& path)
{
    std::unordered_map<int, std::vector<Eigen::Vector3f>> res;
    std::deque<std::pair<double, int>> vec;
	std::ifstream infile;
	infile.open(path.c_str());

	std::string str;
	while (std::getline(infile, str)) 
    {
        std::vector<std::string> strVec;
        simpleSplit(str, strVec, ' ');
        if (5 > strVec.size())
        {
            continue;
        }

        int id = atoi(strVec[0].c_str());
        int num = atoi(strVec[1].c_str());
        if( 0 != (strVec.size() - 2) % 3 )
        {
            continue;
        }

        // for (int i = 0; i < num; i++)
        for (int i = 0; i < num; i+=2)
        {
            res[id].emplace_back(atof(strVec[2 + i * 3].c_str()), atof(strVec[2 + i * 3 + 1].c_str()), atof(strVec[2 + i * 3 + 2].c_str()));
        }
    }

    // float lth = 60;
    float lth = 15;
    float rth = 40;

    std::unordered_map<int, std::vector<Eigen::Vector3f>> tmp_res;

    for (int i = 0; i < frms_.size(); i++)
    {
        Frame& frm = frms_[i];

        int lid = i;
        while((frm.Twc.translation() - frms_[lid].Twc.translation()).norm() < lth && lid != 0)
        {
            lid--;
        }

        int rid = i;
        while((frm.Twc.translation() - frms_[rid].Twc.translation()).norm() < rth && rid != frms_.size() - 1)
        {
            rid++;
        }

        auto& dst = tmp_res[frms_[i].id];
        Sophus::SE3f Twc1 = frms_[i].Twc;
        for (int j = lid; j <= rid; j++)
        {
            auto& ori = res[frms_[j].id];
            if (j == i)
            {
                dst.insert(dst.end(), ori.begin(), ori.end());
            }
            else
            {
                Sophus::SE3f Twc0 = frms_[j].Twc;
                Sophus::SE3f Tc0c1 = Twc1.inverse() * Twc0;
                for(auto& pt : ori)
                {
                    // dst.emplace_back(Tc0c1 * pt + Eigen::Vector3f(0.3, 0.0, 0.3));
                    dst.emplace_back(Tc0c1 * pt);
                }
            }
        }
    }

    // return res;
    return tmp_res;
}

std::vector<std::pair<double, int>> PostProcess::readVideoLog(const std::string& path)
{
    std::vector<std::pair<double, int>> vec;
	std::ifstream infile;
	infile.open(path.c_str());

	std::string str;
	while (std::getline(infile, str)) 
    {
        std::vector<std::string> strVec;
        simpleSplit(str, strVec, ' ');
        if (5 > strVec.size())
        {
            continue;
        }

        if (strVec[2].find("cam_frame") == std::string::npos)
        {
            continue;
        }

        std::string t_str = strVec[0] + '.' + strVec[1];
        double ts = atof(t_str.c_str());

        int id = atoi(strVec[4].c_str());
        vec.emplace_back(ts, id);

        Frame frm;
        frm.id = id;
        frm.ts = ts;
        frms_.push_back(frm);
    }

    return vec;
}

std::vector<Eigen::Vector2f> PostProcess::drawIPMuvs(const std::vector<Eigen::Vector3f>& pts, float limx, float limy)
{
    std::vector<Eigen::Vector2f> uvs;

    if(fabs(limx) > 1e-3 && fabs(limy) > 1e-3)
    {
        for(const auto& it : pts)
        {
            uvs.emplace_back(it(2) / limx * fx + cx, it(0) / limy * fy + cy);
        }
    }

    return uvs;
}


bool PostProcess::loadMask(const std::string& path)
{
    cv::String cvPath = path + "*.bmp";
    std::vector<cv::String> pathVec;
    cv::glob(cvPath, pathVec);
    
    if(pathVec.empty())
    {
        return false;
    }

    int totalCnt = pathVec.size();
    int cnt = 0;
    for(auto& it : pathVec)
    {
        std::vector<std::string> strVec;
        simpleSplit(std::string(it), strVec, '/');
        simpleSplit(strVec.back(), strVec, '.');
        if(strVec.empty())
        {
            std::cout << "load mask : " << cnt++ << " / " << totalCnt << " failed." << std::endl;
            continue;
        }

        int id = atoi(strVec.front().c_str());
        cv::Mat im = cv::imread(it, 0);
        if (im.empty())
        {
            std::cout << "load mask : " << cnt++ << " / " << totalCnt << " failed." << std::endl;
            continue;
        }
        mask_map_[id] = im;
        // std::cout << "load mask : " << cnt++ << " / " << totalCnt << " success." << std::endl;
    }

    if (mask_map_.empty())
    {
        return false;
    }
    

    return true;
}

void PostProcess::setMaskScale(float xs, float ys)
{
    x_scale_ = xs;
    y_scale_ = ys;
    return;
}
