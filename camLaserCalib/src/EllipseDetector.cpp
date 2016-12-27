#include"EllipseDetector.h"
EllipseDetector::EllipseDetector(void)
{
	m_minContourLengthAllowed = 100.0f;

	m_camMat = (Mat_<float>(3,3) << 0,0,0,
		0,0,0,
		0,0,0);
	m_distCoeff = (Mat_<float>(5,1) << 0,0,0,0,0);
    t_fx=427.4473163;
	t_fy=429.4021401;
	t_cx=303.604053;
	t_cy=228.50451;  
	// 3d corner所在坐标为以marker中心为原点
	m_ellipseCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
	m_ellipseCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
	m_ellipseCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
	m_ellipseCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
	pairNum = 0;
	//fopen_s(&fp,"F:\\writen.txt","w"); 
}

EllipseDetector::~EllipseDetector(void)
{
}
void EllipseDetector::processFrame(const Mat& _frame)
{
	m_ellipses.clear();
	findEllipses(_frame, m_ellipses);
}
bool EllipseDetector::findEllipses(const Mat& _frame, vector<EllipseN>& _detectedEllipses)
{
	// RGB转为HSV
	cvtColor(_frame, m_imgHsv, CV_BGR2HSV_FULL);
	// 蓝色过滤
	color_filter(m_imgHsv);
    GaussianBlur(m_imgHsv,m_imgCanny,Size(9,9),0,0);
	Canny(m_imgCanny,m_imgCanny,160,100,3);
	findEllipseContours(m_imgCanny, m_contours, m_imgCanny.cols/5);
	// 	vector<Vec4i> hierarchy;
	// 	Mat contourImg = Mat::zeros(_frame.size(), CV_8UC3);
	// 	for(int i=0; i<m_contours.size(); i++)
	// 	{
	// 		drawContours(contourImg, m_contours, i, Scalar(255,255,255), 2, 8, hierarchy, 0, Point());
	// 	}
	// 	imshow("contours", contourImg);

	// 筛选contours，选择那些又4点围成的contour，得到候选marker
	findEllipseCandidates(m_contours, _detectedEllipses);
	// 	Mat markerCandidateImg = Mat::zeros(_frame.size(), CV_8UC3);
	// 	for(int i=0; i<_detectedMarkers.size(); i++)
	// 	{
	// 		int sizeNum = _detectedMarkers[i].m_points.size();
	// 		for (int j=0; j<sizeNum; j++)
	// 		{
	// 			line(markerCandidateImg, _detectedMarkers[i].m_points[j], _detectedMarkers[i].m_points[(j+1)%sizeNum], Scalar(255,255,255), 2, 8);
	// 		}
	// 	}
	// 	imshow("markerCandidate", markerCandidateImg);

	sortEllipses( _detectedEllipses);
	// 	Mat markerImg = Mat::zeros(_frame.size(), CV_8UC3);
	// 	for(int i=0; i<_detectedMarkers.size(); i++)
	// 	{
	// 		int sizeNum = _detectedMarkers[i].m_points.size();
	// 		for (int j=0; j<sizeNum; j++)
	// 		{
	// 			line(markerImg, _detectedMarkers[i].m_points[j], _detectedMarkers[i].m_points[(j+1)%sizeNum], Scalar(255,255,255), 2, 8);
	// 		}
	// 	}
	// 	imshow("marker", markerImg);

	// 计算姿态
	estimatePosition(_detectedEllipses);
	// 按照id排序
	//std::sort(_detectedEllipses.begin(), _detectedEllipses.end(),std::less<Marker>());
	return false;
}
void EllipseDetector::color_filter(Mat& _imgHsv)
{
	for(int j=0;j < 480;j++)
	{
		uchar* data=_imgHsv.ptr<uchar>(j);
		for(int p=0;p<640*3;p=p+3)
		{
			if( data[p]>150 && data[p]<190 && data[p+1]>60 && data[p+2]<245 && data[p+2]>40 )  //HSV；手动白平衡：3500；
			{ data[p]=255;/*data[p+1]=255;data[p+2]=255;*/ }
			else
			{ data[p]=0;data[p+1]=0;data[p+2]=0; }   
		}
	}
}
void EllipseDetector::findEllipseContours(const Mat& _imgThreshold, vector<vector<Point>>& _contours, int _minContourPointsAllowed)
{
	Mat imgTemp = _imgThreshold.clone();

	vector<vector<Point>> allContours;
	vector<Vec4i> hierarchy;
	findContours(imgTemp, allContours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// 做一个筛选,如果一个contour的点的个数比较少,不是一个好contour
	_contours.clear();
	for (size_t i=0; i<allContours.size(); i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > _minContourPointsAllowed)
		{
			_contours.push_back(allContours[i]);
		}
	}
}

void EllipseDetector::findEllipseCandidates(const vector<vector<Point>>& _contours, vector<EllipseN>& _detectedEllipses)
{
	vector<EllipseN> ellipsePossible;
	EllipseN ellipseTemp;
	for( int i = 0; i< _contours.size(); i++ )     //遍历轮廓
	{ 
		Moments moment; 
		moment=moments(_contours[i],false); //计算轮廓矩
		double u00 = moment.m00;double u01 = moment.m01;double u10 = moment.m10;
		double u20 = moment.mu20;double u11 = moment.mu11;double u02 = moment.mu02;
		double u30 = moment.mu30;double u21 = moment.mu21;double u12 = moment.mu12;double u03 = moment.mu03;
		double I1 = (u02*u20-u11*u11)/pow(u00,4);
		double I2 = (u30*u30*u03*u03-6*u30*u12*u21*u03+4*u30*pow(u12,3)+4*u03*pow(u21,3)-3*u21*u21*u12*u12)/pow(u00,10);
		double I3 = (u20*(u21*u03-u12*u12)-u11*(u30*u03-u21*u12)+u02*(u30*u12-u21*u21))/pow(u00,7); 		
		double Ia = abs(I1-0.006332)-0.0003; 
		double Ib = abs(I2)-0.0000001; 
		double Ic = abs(I3)- 0.0000001;
		double Id=abs(I1-0.006332)-0.0003;
		if ( Ia<=0 && Ib<0 && Ic<0 && _contours[i].size()>100)     //完整椭圆（满足不变矩条件且点数足够多）
		{
			ellipseTemp.Box=fitEllipse(_contours[i]); 
			if(MAX(ellipseTemp.Box.size.width, ellipseTemp.Box.size.height) < (MIN(ellipseTemp.Box.size.width, ellipseTemp.Box.size.height)*2.2))
			{
                ellipsePossible.push_back(ellipseTemp);
			}
		} 
		else                                                     //部分圆弧
		{ 
			convexHull( _contours[i], hull, true );//凸包
			if(hull.size() >=25 && _contours[i].size()>60)  //24,60
			{
				double errOne=0,errAll=0,errOne2=0,errAll2=0,errMean,errMean2;
				double angle;
				ellipseTemp.Box=fitEllipse(hull);
				angle=(ellipseTemp.Box.angle/180)*3.1415926-1.5707963;//
				for(int j =0;j < hull.size();j++)
				{
					errOne=fabs(pow(double ((hull[j].x-ellipseTemp.Box.center.x)*cos(angle)+(hull[j].y-ellipseTemp.Box.center.y)*sin(angle)) / (ellipseTemp.Box.size.height/2),2)+
						pow(double ((hull[j].x-ellipseTemp.Box.center.x)*sin(angle)-(hull[j].y-ellipseTemp.Box.center.y)*cos(angle)) / (ellipseTemp.Box.size.width/2),2)-1);
					errAll+=errOne;
				}
				for(size_t j =0;j < _contours[i].size();j++)
				{
					errOne2=fabs(pow(double ((_contours[i][j].x-ellipseTemp.Box.center.x)*cos(angle)+(_contours[i][j].y-ellipseTemp.Box.center.y)*sin(angle)) / (ellipseTemp.Box.size.height/2),2)+
						pow(double ((_contours[i][j].x-ellipseTemp.Box.center.x)*sin(angle)-(_contours[i][j].y-ellipseTemp.Box.center.y)*cos(angle)) / (ellipseTemp.Box.size.width/2),2)-1);
					errAll2+=errOne2;
				}
				errMean=errAll/hull.size();
				errMean2=errAll2/_contours[i].size();
				if((errMean<0.1)/*&&(errMean2<0.15)*/ && (MAX(ellipseTemp.Box.size.width, ellipseTemp.Box.size.height) < (MIN(ellipseTemp.Box.size.width, ellipseTemp.Box.size.height)*1.2)))
				{	
					ellipsePossible.push_back(ellipseTemp);
				} 
			}//合适的凸包
		}//部分圆弧
	}//遍历轮廓
	vector<pair<int, int>> tooNearCandidates;
	vector<bool> coincideCandidates(ellipsePossible.size(), false);
	vector<bool> ellipseRemoveIndex(ellipsePossible.size(), false);
	if (ellipsePossible.size()>0)
	{
		for (size_t i=0; i<ellipsePossible.size(); i++)
	{
		for (size_t j=i+1; j<ellipsePossible.size(); j++)
		{
		  float ex=abs(ellipsePossible[i].Box.center.x-ellipsePossible[j].Box.center.x);
		  float ey=abs(ellipsePossible[i].Box.center.y-ellipsePossible[j].Box.center.y);
		  float eheight=abs(ellipsePossible[i].Box.size.height-ellipsePossible[j].Box.size.height);
		  float eweight=abs(ellipsePossible[i].Box.size.width-ellipsePossible[j].Box.size.width);
		  float d=sqrt( pow(ex,2)+pow(ey,2) );
			if (d<ellipsePossible[i].Box.size.height*0.6 && eheight<50 && eweight<50)//重合的圆
			{
				coincideCandidates[i]=1;
			}
			if( d<ellipsePossible[i].Box.size.height*0.6 && eheight>ellipsePossible[i].Box.size.height*0.2 && eweight>ellipsePossible[i].Box.size.width*0.2 ) //去掉内圆
			{
				tooNearCandidates.push_back(pair<int, int>(i,j));
			}
		}
	}
	// 选择tooNearCadidates中更小的作为移除的对象
	
	for (size_t i=0; i<tooNearCandidates.size(); i++)
	{
		float length1 = ellipsePossible[tooNearCandidates[i].first].Box.size.height;
		float length2 = ellipsePossible[tooNearCandidates[i].second].Box.size.height;
		ellipseRemoveIndex[(length1>length2) ? tooNearCandidates[i].second : tooNearCandidates[i].first] = true;
	}
	// 去掉内圆及重合的圆得到最终的候选者
	_detectedEllipses.clear();
	for (size_t i=0; i<ellipseRemoveIndex.size(); i++)
	{
		if (!ellipseRemoveIndex[i]&&!coincideCandidates[i])
		{
			_detectedEllipses.push_back(ellipsePossible[i]);
		}
	}
	}//if (ellipsePossible.size()>0)
}

void EllipseDetector::sortEllipses(vector<EllipseN>& _detectedEllipses)
{
	size_t n;
	pairNum = 0;
	if (_detectedEllipses.size()>0)
	{
		rankX(_detectedEllipses);
		float disE;
		for (size_t i=0; i<_detectedEllipses.size()-1; i++)
		{
			disE = sqrt((_detectedEllipses[i].Box.center.x-_detectedEllipses[i+1].Box.center.x)*(_detectedEllipses[i].Box.center.x-_detectedEllipses[i+1].Box.center.x)+(_detectedEllipses[i].Box.center.y-_detectedEllipses[i+1].Box.center.y)*(_detectedEllipses[i].Box.center.y-_detectedEllipses[i+1].Box.center.y));
            if (disE > (3*_detectedEllipses[i].Box.size.width) )
			{
				pairNum = i+1;
			    break;
			}
			else
				pairNum = 0;
		}
		if (pairNum)
		{
			m_ellipsesRight.clear();
			for ( n = 0;n<pairNum;n++)
			{
				m_ellipsesRight.push_back(_detectedEllipses[n]);
			}
			rankY(m_ellipsesRight);
			m_ellipsesLeft.clear();
			for ( n = pairNum;n< _detectedEllipses.size();n++)
			{
				m_ellipsesLeft.push_back(_detectedEllipses[n]);
			}	
			rankY(m_ellipsesLeft);
			_detectedEllipses.clear();
			_detectedEllipses.insert(_detectedEllipses.end(),m_ellipsesRight.begin(),m_ellipsesRight.end());
			_detectedEllipses.insert(_detectedEllipses.end(),m_ellipsesLeft.begin(),m_ellipsesLeft.end());
		}
	    
	}	
	
	//fprintf(fp,"\nTotal ellipses:%d \n",_detectedEllipses.size());

}
void EllipseDetector::rankX(vector<EllipseN>& _detectedE)
{
	size_t i,j;
	EllipseN temp;
	for ( i=0;i<_detectedE.size()-1;i++ )
		{
			for ( j=0;j<_detectedE.size()-1-i;j++ )
			{
				if (_detectedE[j].Box.center.x<_detectedE[j+1].Box.center.x)
				{
					temp = _detectedE[j];
					_detectedE[j] = _detectedE[j+1];
					 _detectedE[j+1] = temp;
				}
			}
		}
}
void EllipseDetector::rankY(vector<EllipseN>& _detectedE)
{
	size_t i,j;
	EllipseN temp;
	for ( i=0;i<_detectedE.size()-1;i++ )
		{
			for ( j=0;j<_detectedE.size()-1-i;j++ )
			{
				if (_detectedE[j].Box.center.y<_detectedE[j+1].Box.center.y)
				{
					temp = _detectedE[j];
					_detectedE[j] = _detectedE[j+1];
					 _detectedE[j+1] = temp;
				}
			}
		}
}
void EllipseDetector::estimatePosition(vector<EllipseN>& _detectedEllipses)
{
	double ellipseAngle,uc,vc,a,b;
	for (size_t i=0; i<_detectedEllipses.size(); i++)
	{
		ellipseAngle = (_detectedEllipses[i].Box.angle/180)*3.1415926-1.5707963;
		uc = _detectedEllipses[i].Box.center.x;
		vc = _detectedEllipses[i].Box.center.y;
		a = _detectedEllipses[i].Box.size.height/2;
		b = _detectedEllipses[i].Box.size.width/2;
        _detectedEllipses[i].m_points.push_back(Point2f(uc+a*cos(ellipseAngle),vc+a*sin(ellipseAngle)));
		_detectedEllipses[i].m_points.push_back(Point2f(uc-b*cos(ellipseAngle),vc+b*sin(ellipseAngle)));
		_detectedEllipses[i].m_points.push_back(Point2f(uc-a*cos(ellipseAngle),vc-a*sin(ellipseAngle)));
		_detectedEllipses[i].m_points.push_back(Point2f(uc+b*cos(ellipseAngle),vc-b*sin(ellipseAngle)));
		_detectedEllipses[i].m_rec.push_back(Point2d( _detectedEllipses[i].m_points[0].x+b*sin(ellipseAngle), _detectedEllipses[i].m_points[0].y-b*cos(ellipseAngle)));//画旋转矩形
		_detectedEllipses[i].m_rec.push_back(Point2d( _detectedEllipses[i].m_points[0].x-b*sin(ellipseAngle), _detectedEllipses[i].m_points[0].y+b*cos(ellipseAngle)));
		_detectedEllipses[i].m_rec.push_back(Point2d( _detectedEllipses[i].m_points[2].x-b*sin(ellipseAngle), _detectedEllipses[i].m_points[2].y+b*cos(ellipseAngle)));
		_detectedEllipses[i].m_rec.push_back(Point2d( _detectedEllipses[i].m_points[2].x+b*sin(ellipseAngle), _detectedEllipses[i].m_points[2].y-b*cos(ellipseAngle)));
		Mat Rvec;
		// 		Mat_<float> Tvec;
		Vec3f Tvec;
		Mat raux, taux;
		// 建立一个3d to 2d的映射
		solvePnP(m_ellipseCorners3d, _detectedEllipses[i].m_points, m_camMat, m_distCoeff, raux, taux);
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);

		// 		Mat_<float> rotMat(3,3);
		Matx33f rotMat;
		Rodrigues(Rvec, rotMat);//旋转向量到旋转矩阵的转换

		_detectedEllipses[i].m_rotation = rotMat.t();
		_detectedEllipses[i].m_translation = -Tvec;
	}
}
