#include <iostream>
#include <Filter/Filter.h>
#include <fstream>
#include <random>
#include <deque>
#include <set>

class Node
{
public:
    Node(){};
    Node(const Eigen::MatrixXi& m, const Eigen::Vector2i& curPos, const Eigen::Vector2i& fixPos, const int dir_id = -1)
    {
        m_ = m;
        w_ = m_.rows();

        curPos_ = curPos;
        fixPos_ = fixPos;

        dir_id_ = dir_id;

        des_.resize(2*w_);
        upDes();
        return;
    }

    void upDes()
    {
        for (int i = 0; i < w_; i++)
        {
            des_(i) = m_.row(i).sum();
        }

        for (int i = 0; i < w_; i++)
        {
            des_(i + w_) = m_.col(i).sum();
        }
    }

    bool isEqual(Node& n)
    {
        auto err = this->des_ - n.des_;
        for (int i = 0; i < err.size(); i++)
        {
            if (err[i] != 0)
            {
                return false;
            }
        }
        
        return true;
    }
    
    int fowradDir(int f_id = -1)
    {
        int x = curPos_[1];
        int y = curPos_[0];

        int fix_x = fixPos_[1];
        int fix_y = fixPos_[0];
            
        std::vector<bool> bm(4, true);
        std::vector<int> s = {rand(), rand(), rand(), rand()};

        if (x - 1 < 0 || (x - 1 == fix_x && y == fix_y) || dir_id_ == 1 || f_id == 0)
        {
            bm[0] = false;
        }

        if (x + 1 > w_ - 1 || (x + 1 == fix_x && y == fix_y) || dir_id_ == 0 || f_id == 1)
        {
            bm[1] = false;
        }

        if (y - 1 < 0 || (x == fix_x && y - 1 == fix_y) || dir_id_ == 3 || f_id == 2)
        {
            bm[2] = false;
        }

        if (y + 1 > w_ - 1 || (x == fix_x && y + 1 == fix_y)  || dir_id_ == 2 || f_id == 3)
        {
            bm[3] = false;
        }

        int id = -1;
        int p = 0;
        for (int i = 0; i < 4; i++)
        {
            if (bm[i] && s[i] > p)
            {
                id = i;
                p = s[i];
            }
        }

        // std::cout << curPos_.transpose() << std::endl;
        // std::cout << bm[0] << " " << bm[1] << " " << bm[2] << " " << bm[3] << std::endl;
        f_id_ = id;

        return id;
    }

    void move(int id)
    {
        dir_id_ = id;
        int x = curPos_[1];
        int y = curPos_[0];

        switch (id)
        {
        case 0:
            {
                int v = m_(y,x);
                m_(y,x) = m_(y, x-1);
                m_(y, x-1) = v;
                curPos_[1]--;
            }
            break;

        case 1:
            {
                int v = m_(y,x);
                m_(y,x) = m_(y, x+1);
                m_(y, x+1) = v;
                curPos_[1]++;
            }
            break;

        case 2:
            {
                int v = m_(y,x);
                m_(y,x) = m_(y-1, x);
                m_(y-1, x) = v;
                curPos_[0]--;
            }
            break;

        case 3:
            {
                int v = m_(y,x);
                m_(y,x) = m_(y+1, x);
                m_(y+1, x) = v;
                curPos_[0]++;
            }
            break;
        
        default:
            break;
        }

        upDes();

        return;
    }

    int w_;
    Eigen::MatrixXi m_;
    Eigen::VectorXi des_;
    Eigen::Vector2i curPos_;
    Eigen::Vector2i fixPos_;
    int dir_id_;
    int f_id_;
};

int calcDist(const Eigen::MatrixXi& m, int N)
{
    int dist = 0;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            int val = m(i,j);

            dist += (abs(int(val/N) - i) + abs(val%N - j));
        }
    }
    return dist;
}


int main(int argc, char const *argv[])
{
    // int w = 3;
    // Eigen::Matrix3i mat;
    // mat << 2, 5, 8, 1, 4, 7, 0, 3, 6;
    // std::cout << "init : " << std::endl;
    // std::cout << mat << std::endl << std::endl;
    // Eigen::Vector2i curPos(0, 2);
    // Eigen::Vector2i fixPos(1, 1);


    int w = 4;
    Eigen::Matrix4i mat;
    mat << 13, 14, 7, 6, 8, 5, 10, 3, 2, 12, 1, 0, 15, 4, 11, 9;
    std::cout << "init : " << std::endl;
    std::cout << mat << std::endl << std::endl;
    Eigen::Vector2i curPos(3, 0);
    // Eigen::Vector2i fixPos(1, 1);
    int fixed = 5;
    Eigen::Vector2i fixPos(int(fixed / w), fixed % w);

    std::deque<Node> nodes;
    Node initNode(mat, curPos, fixPos);
    nodes.push_back(initNode);

    // std::cout << initNode.fowradDir() << std::endl;
    int minDist = INT_MAX;
    Node minNode;

    for (int ite = 0; ite < 50000000; ite++)
    {
        auto& prevNode = nodes.back();
        int m_id = prevNode.fowradDir();

        auto curNode = prevNode;
        curNode.move(m_id);

        int dist = calcDist(curNode.m_,w);
        if (dist < minDist)
        {
            minDist = dist;
            minNode = curNode;
        }
        

        if (0 == dist)
        {
            break;
        }
        

        // bool bLoop = false;
        // for (auto it = nodes.begin(); it != nodes.end();)
        // {
        //     if (bLoop)
        //     {
        //         nodes.erase(it);
        //     }
        //     else
        //     {
        //         bLoop = it->isEqual(curNode);
        //         it++;
        //     }
        // }
        
        // if(bLoop)
        // {
        //     std::cout << nodes.size() << std::endl;
        //     while (-1 == nodes.back().fowradDir(nodes.back().f_id_))
        //     {
        //         nodes.pop_back();
        //     }
        // }
        // else
        // {
        //     nodes.push_back(curNode);
        // }
        nodes.push_back(curNode);
        nodes.pop_front();

        // std::cout << m_id << std::endl;
    }
        std::cout << minNode.m_ << std::endl << std::endl;
        // std::cout << nodes.back().m_ << std::endl << std::endl;

    // int ite = 0;
    // while(1)
    // {
    //     if (ite == 11)
    //     {
    //         // break;
    //     }
    // }


    return 1;
}

// int main(int argc, char const *argv[])
// {
//     std::default_random_engine generator;
//     std::normal_distribution<float> distribution(0.0, 1.0/3);
    

//     Eigen::Vector2f p0(25.0, 0.0);
//     float R0 = 0.0;
//     float w = 1.0 * M_PI / 180.0; // rad / s
//     Eigen::Vector2f v(1.0, 0.3);

//     float fps = 20;
//     float dt = 1.0 / fps;
//     float duration = 50;
//     int updateCnt = fps * duration;

//     float T = 20; //sin

//     std::vector<VTracker::VState> gts;

//     Eigen::Vector2f p_gt = p0;
//     float R_gt = R0;
//     double ts = 0;
//     for (int i = 0; i < updateCnt; i++)
//     {
//         float s = 2.0 * M_PI * ts / T; 
//         Eigen::Vector2f vs;
//         float ws;

//         vs(0) = v(0) + 6.0 * std::sin(s);
//         vs(1) = v(1) + 0.5 * std::sin(0.5 * s);
//         ws = w * std::cos(2.0 * s);
        


//         if (0 != i)
//         {
//             R_gt += (ws * dt);
//             p_gt += (vs * dt);
//             ts += dt;
//         }


//         VTracker::VState gt;
//         gt.omega = ws;
//         gt.v_V = vs;
//         gt.p_V = p_gt;
//         gt.theta = R_gt;
//         gt.ts = ts;
//         gt.w_V = 1.5;
//         gts.push_back(gt);
//     }

//     // obs
//     float v_w = 1.5;
//     std::vector<float> vec_pt = {0.1,
//                                  0.12,
//                                  0.2,
//                                  0.35,
//                                  0.39,
//                                  0.43,
//                                  0.52,
//                                  0.55,
//                                  0.6,
//                                  0.67,
//                                  0.75,
//                                  0.82,
//                                  0.94,
//                                  0.99};

//     VTracker::Filter filter;
//     std::vector<VTracker::Obs> obs(gts.size());
//     for (int i = 0; i < gts.size(); i++)
//     {
//         obs[i].ts = gts[i].ts;

//         {
//             Eigen::Vector2f pl(0, - 0.5 * v_w);
//             Eigen::Vector2f pr(0,   0.5 * v_w);
//             Eigen::Vector2f pwl = VTracker::rotMatrix(gts[i].theta) * pl + gts[i].p_V;
//             Eigen::Vector2f pwr = VTracker::rotMatrix(gts[i].theta) * pr + gts[i].p_V;
//             Eigen::Vector2f pcl = pwl / pwl(0);
//             Eigen::Vector2f pcr = pwr / pwr(0);

//             float rl = distribution(generator);
//             float ul = pcl(1) + (2.0 * rl) / 1458.0;

//             float rr = distribution(generator);
//             float ur = pcr(1) + (2.0 * rr) / 1458.0;

//             float w = ur - ul;
//             obs[i].wv_img = w;
//             obs[i].ur = ur;
//             obs[i].ul = ul;

//             float r_dist = distribution(generator);
//             if (gts[i].p_V(0) < 50.0)
//             {
//                 obs[i].dist = (1 + 0.3 * r_dist) * gts[i].p_V(0) ;
//             }
//         }

//         for (int j = 0; j < vec_pt.size(); j++)
//         {
//             Eigen::Vector2f p(0, (vec_pt[j] - 0.5) * v_w);
//             Eigen::Vector2f pw = VTracker::rotMatrix(gts[i].theta) * p + gts[i].p_V;
//             Eigen::Vector2f pc = pw / pw(0);

//             float r = distribution(generator);
//             float nu = pc(1) + (0.5 * r) / 1458.0;
//             obs[i].us.emplace_back(j, nu);
//         }

//         if (0 == i)
//         {
//             VTracker::VState initState;
//             initState.p_V = Eigen::Vector2f(15, 0);
//             initState.theta = 0;
//             initState.omega = 0.0 * M_PI / 180.0;
//             initState.v_V = Eigen::Vector2f(0.0,0.0);
//             initState.a_V = Eigen::Vector2f(0.0,0.0);
//             initState.w_V = 1.7;
//             initState.ts = ts;

//             filter.initFilterState(initState, obs[i]);
//         }
//         else
//         {
//             filter.obsCallBack(obs[i]);
//             auto state = filter.getState();

//             static std::ofstream of_gt("gt.csv");
//             static std::ofstream of_res("res.csv");

//             of_gt << gts[i].p_V[0] << " " << gts[i].p_V[1] << " " << gts[i].v_V[0] << " " << gts[i].v_V[1] << " " << gts[i].theta << " " << gts[i].omega << std::endl;
//             of_res << state.p_V[0] << " " << state.p_V[1] << " " << state.v_V[0] << " " << state.v_V[1] << " " << state.theta << " " << state.omega << " " << state.w_V << std::endl;
//         }
//     }

//     return 0;
// }


