#pragma once
#include <opencv2/opencv.hpp>
#include "data.hpp"
// #include <string>

template <typename T>
class circularQueue
{
public:
    // circularQueue(const int& n);
    bool init(const int &n);
    bool quit(void);

    int length(void) const;
    bool isEmpty(void) const;
    bool isFull(void) const;
    bool enqueue(const T &elem);
    T dequeue(void);
    T &get(const int &n) const;
    bool clear(void);
    bool remove(const int &n);

    T &operator[](const int &n);

private:
    T *arr = nullptr;
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

    pose_pack coordinateTrans(const cv::Point3f &targetPoint, const std::vector<cv::Point2f> &inputPoints, my_time &, my_data &); // 没理解错的话应该是Armor类调用我，即识别一个点转化一次
    pose_pack traceCal(my_time &, my_data &);

    void leastSquare(my_data &); // 最小二乘求最优解

    void test(void);

private:
    // bool ifReceived = 0;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    // cv::Mat R2;//旋转矩阵

    std::vector<cv::Point3f> PW3D_Small{{-0.065, 0.0295, 0}, {0.065, 0.0295, 0}, {0.065, -0.0295, 0}, {-0.065, -0.0295, 0}}; // 小装甲板参数
    // std::vector<cv::Point3f> PW3D_Small {{0, -0.065, 0.0295}, {0, 0.065, 0.0295}, {0, 0.065, -0.0295}, {0, -0.065, -0.0295}};//小装甲板参数
    std::vector<cv::Point3f> PW3D_Big{{-0.1125, 0.0295, 0}, {0.1125, 0.0295, 0}, {0.1125, -0.0295, 0}, {-0.1125, -0.0295, 0}}; // 大装甲板参数

    // std::vector<cv::Point2f> PC2D;
    // std::vector<double> rvec;//旋转向量
    // std::vector<double> tvec;//平移向量

    // double length = 0.103771; // 相机坐标系转云台坐标系时z轴的平移量，需要机械测量
    // double height = 0.042518; // 相机坐标系转云台坐标系时x轴的平移量

    double length = 0.13362; // 相机坐标系转云台坐标系时z轴的平移量，需要机械测量
    double height = 0.08539; // 相机坐标系转云台坐标系时x轴的平移量        //英雄

    double v0_small = 15; // 小弹丸的初速度，这个也需要测
    double k_small = 7.1655E-5;
    double m_small = 0.0032;
    double v0_big = 15; // 大弹丸的初速度，这个也需要测
    double k_big = 4.5857E-4;
    double m_big = 0.0041;

    unsigned long long int t_hit = 0;   // 弹丸击中装甲板所需的时间，这个不需要动（单位：毫秒）
    unsigned long long int t_delay = 0; // 这个是电控的控制延迟（即云台转动需要的时间），需要调
    // unsigned long long int delta_t = 0; // 这个是调用我的周期，一定要调！！！

    int queue_length = 10; // 队列长度，即最小二乘时样本的个数

    circularQueue<pose_pack> pack_pre;

    double yaw_result;
    double pitch_result; // 这里用来存放最终的预测结果

    struct
    {
        double x;
        double y;
        double z;
        double xy_plane_distance;
        double yaw;

    } target;

    // cv::Point3f outputPoint;
};

#include <stdio.h>

// circularQueue::circularQueue(const int& n)
// {
//     arr = new double[n + 1];
//     front = 0;
//     rear = 0;
//     max_size = n + 1;
// }

template <typename T>
bool circularQueue<T>::init(const int &n)
{
    if (arr == 0)
        arr = new T[n + 1];
    front = 0;
    rear = 0;
    max_size = n + 1;
    return 1;
}

template <typename T>
bool circularQueue<T>::quit(void)
{
    delete[] arr;
    return 1;
}

template <typename T>
int circularQueue<T>::length(void) const
{
    return (rear - front + max_size) % max_size;
}

template <typename T>
bool circularQueue<T>::isEmpty(void) const
{
    return rear == front;
}

template <typename T>
bool circularQueue<T>::isFull(void) const
{
    return (rear + 1) % max_size == front;
}

template <typename T>
bool circularQueue<T>::enqueue(const T &elem)
{
    if (isFull())
    {
        std::cout << "Full already!" << std::endl;
        return 0;
    }
    else
    {
        arr[rear] = elem;
        rear = (rear + 1) % max_size;
        return 1;
    }
}

template <typename T>
T circularQueue<T>::dequeue(void)
{
    if (length() == 0)
    {
        T tmp;
        return tmp;
    }
    else
    {
        T tmp = arr[front];
        // arr[front] = 0;
        front = (front + 1) % max_size;
        return tmp;
    }
}

template <typename T>
T &circularQueue<T>::operator[](const int &n)
{
    // if (n + 1 >= length() || n < 0)
    // {
    //     std::cout << "Out of scope!" << std::endl;
    //     return arr[rear];
    // }
    // else
    // {
    return arr[(front + n) % max_size];
    // }
}

template <typename T>
T &circularQueue<T>::get(const int &n) const
{
    return arr[(rear + max_size - n) % max_size];
}

template <typename T>
bool circularQueue<T>::clear(void) // 清空
{
    while (!isEmpty())
    {
        dequeue();
    }
    return 1;
}

template <typename T>
bool circularQueue<T>::remove(const int &n) // 删掉几个
{
    if (n < 0)
    {
        std::cout << "Num of remove error!" << std::endl;
        return 0;
    }
    else if (n >= length())
    {
        clear();
        return 1;
    }
    else
    {
        for (int tmp = 0; tmp < n; ++tmp)
        {
            dequeue();
        }
        return 1;
    }
}

bool TargetSolver::init(void)
{
    yaw_result = 0;
    pitch_result = 0; // 这里用来存放最终的预测结果

    target.x = 0;
    target.y = 0;
    target.z = 0;
    target.xy_plane_distance = 0;
    target.yaw = 0;

    pack_pre.quit();
    pack_pre.init(queue_length);
    return 1;
}

bool TargetSolver::readParas(std::string fileName)
{
    int rows_cam = 0;
    int cols_cam = 0;
    int rows_dist = 0;
    int cols_dist = 0;

    cv::FileStorage fsread(fileName, cv::FileStorage::READ);

    if (!fsread.isOpened())
        return 0;

    fsread["camera_internal_matrix"]["rows"] >> rows_cam;
    fsread["camera_internal_matrix"]["cols"] >> cols_cam;
    fsread["distortion_coeff"]["rows"] >> rows_dist;
    fsread["distortion_coeff"]["cols"] >> cols_dist;

    // cv::FileNode cam = fsread["camera_internal_matrix"]["data"];
    // cv::Mat e(3, 3, CV_64FC1);
    // printf("%d\n", cam);
    // cam[0] >> e;//失败的尝试1

    // cv::FileNode fn4 = fsread["camera_internal_matrix"];
    // cv::FileNode fn4l = fn4["data"];
    // std::cout << fn4l[0] << std::endl;
    // fn4l[0] >> cameraMatrix;//失败的尝试2

    // cameraMatrix.resize(rows_cam, cols_cam);
    // distCoeffs.resize(rows_dist, cols_dist);
    //  fsread["camera_internal_matrix"]["data"] >> cameraMatrix;
    //  fsread["distortion_coeff"]["data"] >> distCoeffs;//失败的尝试3

    // cv::Mat c(rows_cam, cols_cam, CV_32FC1, cv::Scalar::all(0));
    // fsread["camera_internal_matrix"]["data"] >> c;//失败的尝试4

    cv::FileNode fn1 = fsread["camera_internal_matrix"];
    cv::FileNode fn2 = fn1["data"];
    cv::FileNodeIterator fn2_iter = fn2.begin();
    double tmp[9];
    for (int i = 0; i < 9; ++i)
    {
        tmp[i] = (*fn2_iter);
        ++fn2_iter;
    }
    cameraMatrix = (cv::Mat_<double>(3, 3) << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8]);

    fn1 = fsread["distortion_coeff"];
    fn2 = fn1["data"];
    fn2_iter = fn2.begin();
    for (int i = 0; i < 5; ++i)
    {
        tmp[i] = (*fn2_iter);
        ++fn2_iter;
    }
    distCoeffs = (cv::Mat_<double>(1, 5) << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4]);

    // std::cout << distCoeffs << std::endl;
    return 1;
}

// bool TargetSolver::receiveDta(void)
// {}

pose_pack TargetSolver::coordinateTrans(const cv::Point3f &targetPoint, const std::vector<cv::Point2f> &inputPoints, my_time &mt, my_data &data) // 没理解错的话应该是Armor类调用我，即识别一个点转化一次
{
    std::vector<double> rvec; // 旋转向量
    std::vector<double> tvec; // 平移向量
    cv::Mat R;
    /*第一步：获取想要的参数*/
    // cv::RotatedRect Rect_1, Rect_2;//得改，因为这应该是从Armor里提取的
    //  std::vector<cv::Point2f> P2D;//从Armor里取4个点以及顺序
    //  P2D.push_back(cv::Point2f(140, 205));
    //  P2D.push_back(cv::Point2f(182, 204));
    //  P2D.push_back(cv::Point2f(182, 222));
    //  P2D.push_back(cv::Point2f(141, 223));

    // P2D.push_back(cv::Point2f(0.2, 0.2));
    // P2D.push_back(cv::Point2f(0.6, 0.2));
    // P2D.push_back(cv::Point2f(0.6, 0.4));
    // P2D.push_back(cv::Point2f(0.2, 0.4));

    bool isBig = 0;

    /*第二步：获取旋转向量以及平移向量*/
    if (0 == isBig)
    {
        cv::solvePnP(PW3D_Small, inputPoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    else
    {
        cv::solvePnP(PW3D_Big, inputPoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }

    // std::cout << "tvec: ";
    // for (auto &tmp : tvec)
    // {
    //     std::cout << tmp << ' ';
    // }
    // std::cout << std::endl;

    /*第三步：旋转向量转化为旋转矩阵*/
    cv::Rodrigues(rvec, R);

    /*第四步：矩阵相乘，将输入点在世界系的坐标转化为在相机系的坐标*/
    cv::Mat P(4, 4, CV_64FC1);
    // cv::Mat x_input(4, 1, CV_64FC1);
    cv::Mat x_output(4, 1, CV_64FC1);

    x_output.at<double>(0, 0) = targetPoint.x;
    x_output.at<double>(1, 0) = targetPoint.y;
    x_output.at<double>(2, 0) = targetPoint.z;
    x_output.at<double>(3, 0) = 1;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P.at<double>(i, j) = R.at<double>(i, j);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        P.at<double>(i, 3) = tvec[i];
    }
    for (int i = 0; i < 4; ++i)
    {
        if (3 == i)
        {
            P.at<double>(3, i) = 1;
        }
        else
        {
            P.at<double>(3, i) = 0;
        }
    }

    x_output = P * x_output;
    // std::cout << "298: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << std::endl;
    /*第五步：矩阵相加并将坐标系绕x轴逆时针旋转90度，将目标点在相机系转为在云台系*/
    x_output.at<double>(2, 0) += length; // z的变换，length还需要和机械同学商量>
    x_output.at<double>(1, 0) -= height; // y的变换，height还需要和机械同学商量
    // std::cout << "302: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << std::endl;

    double tmp = -x_output.at<double>(1, 0);
    x_output.at<double>(1, 0) = x_output.at<double>(2, 0);
    x_output.at<double>(2, 0) = tmp; // 云台系1到云台系2

    // std::cout << "308: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << "yaw = " << 180 * atan2(x_output.at<double>(0, 0), x_output.at<double>(1, 0)) / acos(-1.0) << std::endl;

    /*第六步：将该点在云台系的坐标转换为在大地坐标系的坐标*/

    // double yaw = -5.5333 * acos(-1.0) / 180, pitch = 0.7388 * acos(-1.0) / 180, roll = -0.0277 * acos(-1.0) / 180;

    // get(yaw, pitch, roll);//这里需要通信那边的电控发给视觉的实时yaw，pitch，roll的值
    // printf("\n%ld\n",data.ts.GetTimeStamp(mt).time_ms);
    pose_pack pack = data.get_info(data.ts.GetTimeStamp(mt));

    printf("%lf\t%lf\n",pack.ptz_pitch,pack.ptz_yaw);

    cv::Mat YAW = (cv::Mat_<double>(4, 4) << cos(pack.ptz_yaw*acos(-1.0)/180.0), -sin(pack.ptz_yaw*acos(-1.0)/180.0), 0, 0, sin(pack.ptz_yaw*acos(-1.0)/180.0), cos(pack.ptz_yaw*acos(-1.0)/180.0), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    cv::Mat PITCH = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, cos(pack.ptz_pitch*acos(-1.0)/180.0), -sin(pack.ptz_pitch*acos(-1.0)/180.0), 0, 0, sin(pack.ptz_pitch*acos(-1.0)/180.0), cos(pack.ptz_pitch*acos(-1.0)/180.0), 0, 0, 0, 0, 1);
    cv::Mat ROLL = (cv::Mat_<double>(4, 4) << cos(pack.ptz_roll*acos(-1.0)/180.0), 0, sin(pack.ptz_roll*acos(-1.0)/180.0), 0, 0, 1, 0, 0, -sin(pack.ptz_roll*acos(-1.0)/180.0), 0, cos(pack.ptz_roll*acos(-1.0)/180.0), 0, 0, 0, 0, 1);
    // printf("\n2\n");
    x_output = YAW * PITCH * ROLL * x_output;
    /*第七步：返回结果*/
    target.x = x_output.at<double>(0, 0);
    target.y = x_output.at<double>(1, 0);
    target.z = x_output.at<double>(2, 0);
    target.xy_plane_distance = sqrt(target.x * target.x + target.y * target.y);
    target.yaw = -atan2(target.y, target.x);
    /*这里还得考虑边缘情况*/
    /*测试区*/
    // std::cout << "大地坐标系：";
    std::cout << "x = " << x_output.at<double>(0, 0) << ", y = " << x_output.at<double>(1, 0) << ", z = " << x_output.at<double>(2, 0) << std::endl;
    std::cout<<"distance:"<<powf32(x_output.at<double>(0, 0),2)+powf32(x_output.at<double>(1, 0),2)<<std::endl;

    data.sendTargetDataPack.pred_pitch = this->pitch_result;
    data.sendTargetDataPack.pred_yaw = this->yaw_result;
    return this->traceCal(mt, data);
}

pose_pack TargetSolver::traceCal(my_time &mt, my_data &md)
{
    double x = target.xy_plane_distance;
    double y = target.z;
    double tan_theta = 0.5; // 初始值设置成0.5
    double yaw = 0, pitch = 0;

    // 方法一：不动点迭代，迭代5次
    //  for (int i = 0; i < 5; ++i)
    //  {
    //      tan_theta = (k_small * v0_small * y / (m_small * v0_small * (exp(k_small * x / m_small) - 1))) + m_small * 9.8 * (exp(k_small * x / m_small) - 1) * (1 + tan_theta * tan_theta) / (2 * k_small * v0_small * v0_small);
    //  }

    // 方法二：直接接二元一次方程
    tan_theta = (1 - sqrt(1 - 2 * 9.8 * y / (v0_small * v0_small))) * k_small * v0_small * v0_small / (m_small * 9.8 * (exp(k_small * x / m_small) - 1)); // 这里最开始的1 - 待定，可能是1 +

    yaw = -180 * atan2(target.x, target.y) / acos(-1.0);
    pitch = 180 * atan2(tan_theta, 1) / acos(-1.0);
    // std::cout << "yaw: " << yaw << std::endl;
    // std::cout << "pitch: " << pitch << std::endl;

    t_hit = 1000 * m_small * (exp(k_small * x / m_small) - 1) / (k_small * v0_small * cos(atan2(tan_theta, 1)));
    // printf("entre\n");
    // if (pack_pre.isFull())
    // {
    //     leastSquare(md); // 先算一步
    //     pack_pre.dequeue();
    // }
    // printf("end\n");

    pose_pack tmp;
    tmp.ptz_yaw = yaw;
    tmp.ptz_pitch = pitch;
    tmp.pack_time = mt;
    // md.write_pack(tmp);
    return tmp;
    // pack_pre.enqueue(tmp);
}

void TargetSolver::leastSquare(my_data &md) // 最小二乘求最优解
{
    double b_yaw = 0, a_yaw = 0, sigma_xy_yaw = 0, sigma_x_yaw = 0, sigma_y_yaw = 0, sigma_xx_yaw = 0;
    double b_pitch = 0, a_pitch = 0, sigma_xy_pitch = 0, sigma_x_pitch = 0, sigma_y_pitch = 0, sigma_xx_pitch = 0;
// printf("entre\n");

    for (int i = 0; i < queue_length; ++i)
    {
        sigma_xy_yaw += pack_pre[i].pack_time.time_ms * pack_pre[i].ptz_yaw;
        sigma_x_yaw += pack_pre[i].pack_time.time_ms;
        sigma_y_yaw += pack_pre[i].ptz_yaw;
        sigma_xx_yaw += pack_pre[i].pack_time.time_ms * pack_pre[i].pack_time.time_ms;

        sigma_xy_pitch += pack_pre[i].pack_time.time_ms * pack_pre[i].ptz_pitch;
        sigma_x_pitch += pack_pre[i].pack_time.time_ms;
        sigma_y_pitch += pack_pre[i].ptz_pitch;
        sigma_xx_pitch += pack_pre[i].pack_time.time_ms * pack_pre[i].pack_time.time_ms;
    }
// printf("entre1\n");

    b_yaw = (sigma_xy_yaw - sigma_x_yaw * sigma_y_yaw / queue_length) / (sigma_xx_yaw - sigma_x_yaw * sigma_x_yaw / queue_length);
    a_yaw = (sigma_y_yaw - b_yaw * sigma_x_yaw) / queue_length;

    b_pitch = (sigma_xy_pitch - sigma_x_pitch * sigma_y_pitch / queue_length) / (sigma_xx_pitch - sigma_x_pitch * sigma_x_pitch / queue_length);
    a_pitch = (sigma_y_pitch - b_pitch * sigma_x_pitch) / queue_length;
// printf("entre2\n");

    // yaw_result = b_yaw * (queue_length + t_hit / delta_t + t_delay / delta_t) + a_yaw;
    // pitch_result = b_pitch * (queue_length + t_hit / delta_t + t_delay / delta_t) + a_pitch;
// printf("entre3\n");

    yaw_result = b_yaw * (md.ts.GetTimeStamp().time_ms + t_hit + t_delay) + a_yaw;
    pitch_result = b_pitch * (md.ts.GetTimeStamp().time_ms + t_hit + t_delay) + a_pitch;
}

void TargetSolver::test(void)
{
    double x = target.xy_plane_distance;
    double y = target.z;

    double tan_theta, pitch, yaw;

    // 方法一：不动点迭代，迭代5次
    //  for (int i = 0; i < 5; ++i)
    //  {
    //      tan_theta = (k_small * v0_small * y / (m_small * v0_small * (exp(k_small * x / m_small) - 1))) + m_small * 9.8 * (exp(k_small * x / m_small) - 1) * (1 + tan_theta * tan_theta) / (2 * k_small * v0_small * v0_small);
    //  }
    // 方法二：直接接二元一次方程
    yaw = -180 * atan2(target.x, target.y) / acos(-1.0);
    tan_theta = (1 - sqrt(1 - 2 * 9.8 * y / (v0_small * v0_small))) * k_small * v0_small * v0_small / (m_small * 9.8 * (exp(k_small * x / m_small) - 1)); // 这里最开始的1 - 待定，可能是1 +
    pitch = 180 * atan2(tan_theta, 1) / acos(-1.0);
    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
}
