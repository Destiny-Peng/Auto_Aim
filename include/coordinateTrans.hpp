#pragma once
#include <opencv2/opencv.hpp>
//#include <string>


class circularQueue
{
public:
    //circularQueue(const int& n);
    bool init(const int& n);
    bool quit(void);

    int length(void) const;
    bool isEmpty(void) const;
    bool isFull(void) const;
    bool enqueue(const double& elem);
    double dequeue(void);
    double& get(const int& n) const;
    bool clear(void);
    bool remove(const int& n);

    double& operator[](const int& n);
private:
    double* arr = 0;
    int front;
    int rear;

    int max_size;
};


class TargetSolver
{
public:
    bool init(void);
    bool readParas(std::string fileName);
    // bool receiveDta(void);


    void coordinateTrans(const cv::Point3f& targetPoint, const std::vector<cv::Point2f>& inputPoints);//没理解错的话应该是Armor类调用我，即识别一个点转化一次
    void traceCal(void);


    void leastSquare(void);//最小二乘求最优解

    void test(void);
private:
    //bool ifReceived = 0;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;    
    //cv::Mat R2;//旋转矩阵

    std::vector<cv::Point3f> PW3D_Small {{-0.065, 0.0295, 0}, {0.065, 0.0295, 0}, {0.065, -0.0295, 0}, {-0.065, -0.0295, 0}};//小装甲板参数
    //std::vector<cv::Point3f> PW3D_Small {{0, -0.065, 0.0295}, {0, 0.065, 0.0295}, {0, 0.065, -0.0295}, {0, -0.065, -0.0295}};//小装甲板参数
    std::vector<cv::Point3f> PW3D_Big {{-0.1125, 0.0295, 0}, {0.1125, 0.0295, 0}, {0.1125, -0.0295, 0}, {-0.1125, -0.0295, 0}};//大装甲板参数

    //std::vector<cv::Point2f> PC2D;
    //std::vector<double> rvec;//旋转向量
    //std::vector<double> tvec;//平移向量

    double length = 0.103771;//相机坐标系转云台坐标系时z轴的平移量，需要机械测量
    double height = 0.042518;//相机坐标系转云台坐标系时x轴的平移量

    double v0_small = 15;//小弹丸的初速度，这个也需要测
    double k_small = 7.1655E-5;
    double m_small = 0.0032;
    double v0_big = 15;//大弹丸的初速度，这个也需要测
    double k_big = 4.5857E-4;
    double m_big = 0.0041;

    double t_hit = 0;//弹丸击中装甲板所需的时间，这个不需要动
    double t_delay = 0;//这个是电控的控制延迟（即云台转动需要的时间），需要调
    double delta_t = 0;//这个是调用我的周期，一定要调！！！

    int queue_length = 10;//队列长度，即最小二乘时样本的个数

    circularQueue yaw_pre;
    circularQueue pitch_pre;

    double yaw_result;
    double pitch_result;//这里用来存放最终的预测结果

    struct
    {
        double x;
        double y;
        double z;
        double xy_plane_distance;
        double yaw;
        
    } target;

    //cv::Point3f outputPoint;

};