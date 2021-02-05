#include "viewer.h"

Viewer::~Viewer()
{
    stop();
    return;
}

Viewer::Viewer()
{
	win_name_ = "viewer";
	win_w_ = 640;
	win_h_ = 480;
    dur_ = 0.049;
	curr_t_ = 0.0;
	bShowIpm_ = false;

    bStop_ = false;
	srand(time(NULL));


    return;
}

bool Viewer::isStop()
{
    return bStop_;
}

void Viewer::init()
{
	// create a window and bind its context to the main thread
	pangolin::CreateWindowAndBind(win_name_, 2 * win_w_, win_h_);

	// enable depth
	glEnable(GL_DEPTH_TEST);

	// unset the current context from the main thread
	pangolin::GetBoundWindow()->RemoveCurrent();

	th_draw_ = std::thread(&Viewer::run, this);

    return;
}

void Viewer::run()
{

	// fetch the context and bind it to this thread
	pangolin::BindToContext(win_name_);

	// Define Projection and initial ModelView matrix
	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(win_w_, win_h_, 420, 420, 320, 240, 0.2, 1e10),
		pangolin::ModelViewLookAt(0.0, -10.0, -15.0, 0, 0, 0, 0.0, -1.0, 0.0)
	);

	// Create Interactive View in window
	pangolin::Handler3D handler(s_cam);
	pangolin::View &d_cam = pangolin::Display("cam0")
		.SetAspect((float) win_w_ / (float) win_h_)
		.SetHandler(&handler);

	// Add named OpenGL viewport to do imgshow
	pangolin::View &d_cam1 = pangolin::Display("cam1")
		.SetAspect((float) win_w_ / (float) win_h_);

	// create Multi-Display
	pangolin::Display("multi")
		.SetBounds(0.0, 1.0, 0.0, 1.0)
		.SetLayout(pangolin::LayoutEqualHorizontal)
		.AddDisplay(d_cam)
		.AddDisplay(d_cam1);

	pangolin::RegisterKeyPressCallback('o', [&]
	{ bShowIpm_ = !bShowIpm_; });

    std::unordered_map<std::string, std::deque<std::pair<double, std::vector<Eigen::Vector3f>>>>  draw_pts;
    std::deque<cv::Mat> draw_imgs;

	while (!pangolin::ShouldQuit() && !bStop_) 
    {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        std::unordered_map<std::string, std::deque<std::pair<double, std::vector<Eigen::Vector3f>>>>  tmp_pts;
        {
            pts_mtx_.lock();
            tmp_pts.swap(pts_);
            pts_mtx_.unlock();
        }

        for(auto& group : tmp_pts)
        {
            std::string type = group.first;
            draw_pts[type].insert(draw_pts[type].end(), group.second.begin(), group.second.end());
			double ts = group.second.back().first;
			if (ts > curr_t_)
			{
				curr_t_ = ts;
			}
        }

		std::unordered_map<std::string, std::deque<std::pair<double, std::vector<Eigen::Vector3f>>>>::iterator ite;
		for(ite = draw_pts.begin(); ite != draw_pts.end(); ite)
		{
			auto& group = *ite;
			if(group.second.empty())
			{
				ite = draw_pts.erase(ite);
			}
			else
			{
				std::string type = group.first;
				if(type.find("lidar") != std::string::npos)
				{
   	 	        	while (1)
   		         	{
						if (draw_pts[type].empty() || curr_t_ - draw_pts[type].front().first < dur_)
						{
							break;
						}
   		         	    draw_pts[type].pop_front();
  	 	         	}
				}
				else if (type.find("vio") != std::string::npos)
				{
 	  	         	while (1)
	   	         	{
						if (draw_pts[type].empty() || curr_t_ - draw_pts[type].front().first < dur_)
						{
							break;
						}
	   	         	    draw_pts[type].pop_front();
	   	         	}
				}

				glPointSize(3);
				Eigen::Vector3f c = colors_[type];
				glColor3f(c[0], c[1], c[2]);

				// if(type.find("lidar") != std::string::npos)
				// {
				// 	glColor3f(1.0, 0.0, 0.0);
				// }

				// if(type.find("vio") != std::string::npos)
				// {
				// 	glColor3f(0.0, 1.0, 0);
				// }

				for(auto& p3ds : group.second)
				{
					pangolin::glDrawPoints(p3ds.second);
				}

				ite++;
			}
		}

		//draw center
		glPointSize(5);
		glColor3f(1.0, 0, 0.0);
		glBegin(GL_POINTS);
		glVertex3f(0, 0, 0);
		glEnd();
		//draw center end

		std::deque<cv::Mat> tmp_imgs;
		bool bImgUpdate = false;
		{
			img_mtx_.lock();
			if (!imgs_.empty()) {
				draw_imgs.swap(imgs_);
				imgs_.swap(tmp_imgs);
				bImgUpdate = true;
			}
			img_mtx_.unlock();
		}

		if (!draw_imgs.empty()) {
			cv::Mat &im = draw_imgs.back();
			if (!im.empty()) {
				d_cam1.Activate();
				drawImage(im, bImgUpdate);
			}
		}

        pangolin::FinishFrame();
    }


	// unset the current context from the main thread
	pangolin::GetBoundWindow()->RemoveCurrent();

	// close window
	pangolin::DestroyWindow(win_name_);

    bStop_ = true;

    return;
}

void Viewer::stop()
{
	bStop_ = true;
	if (th_draw_.joinable()) {
		th_draw_.join();
	}
	return;
}


void Viewer::insertPoints(const double& time, const std::vector<Eigen::Vector3f>& p3ds, std::string str)
{

    pts_mtx_.lock();

	if (!colors_.count(str))
	{
		float c0 = 0.8 * float(rand() % 1000) / 1000.0  + 0.2;
		float c1 = 0.8 * float(rand() % 1000) / 1000.0  + 0.2;
		float c2 = 0.8 * float(rand() % 1000) / 1000.0  + 0.2;
		colors_[str] = Eigen::Vector3f(c0, c1, c2);
	}

    pts_[str].emplace_back(time, p3ds);

    pts_mtx_.unlock();

    return;
}


void Viewer::insertImg(const cv::Mat& im)
{
    img_mtx_.lock();
    imgs_.emplace_back(im);
    img_mtx_.unlock();

    return;
}

void Viewer::drawImage(cv::Mat &im, bool &bUpdate)
{
	int w = im.cols;
	int h = im.rows;
	static GLuint tid = 0;
	if (0 == tid) {
		glGenTextures(1, &tid);
		glBindTexture(GL_TEXTURE_2D, tid);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGR, GL_UNSIGNED_BYTE, im.data);
	}
	else {
		glBindTexture(GL_TEXTURE_2D, tid);
		if (bUpdate) {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, im.data);
		}
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	GLfloat sq_vert[] = {-1, -1, 1, -1, 1, 1, -1, 1};
	glVertexPointer(2, GL_FLOAT, 0, sq_vert);
	glEnableClientState(GL_VERTEX_ARRAY);

	GLfloat sq_tex[] = {0, 1, 1, 1, 1, 0, 0, 0};
	glTexCoordPointer(2, GL_FLOAT, 0, sq_tex);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glEnable(GL_TEXTURE_2D);

	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	glDisable(GL_TEXTURE_2D);
}