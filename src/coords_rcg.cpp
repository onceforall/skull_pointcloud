#include "coords_rcg.h"

vector<Vec3f> getcoords(string picpath)
{
    Mat src,gray;
    src=imread(picpath,0);
    cout<<"get points"<<endl;
    adaptiveThreshold(src,gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,3,5);
    //cvtColor(src,gray,COLOR_BGR2GRAY);
    GaussianBlur(gray,gray,Size(3,3),2,2);
    imshow("origin",gray);
    vector<Vec3f> circles;
    HoughCircles(gray,circles,HOUGH_GRADIENT,1,100,100,30,5,20);
    
    for(size_t i=0;i<circles.size();i++)
    {
        Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        int radius=cvRound(circles[i][2]);
        circle(src,center,3,Scalar(255,0,0),-1,8,0);
        circle(src,center,radius,Scalar(155,50,255),3,8,0);
    }
   
    vector<Mat> res;
     Mat Cameramatrix =(Mat_<double>(3,3)<<1959.924848637667, 0, 545.1486968090578,0,1842.754424436342, 756.0013735321402, 0, 0,1);//直接赋初始值的方法
    for(auto x:circles)
    {
        Mat point=(Mat_<double>(3,1)<<x[0], x[1], 1);
        Mat coord=Cameramatrix.inv()*point;
        cout<<coord.at<double>(0)<<' '<<coord.at<double>(1)<<' '<<coord.at<double>(2)<<endl;
        res.push_back(coord);
    }
    imshow("result",src);
    waitKey(0);

    return circles;
}
