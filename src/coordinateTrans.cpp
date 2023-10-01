#include <coordinateTrans.hpp>
#include <stdio.h>


// circularQueue::circularQueue(const int& n)
// {
//     arr = new double[n + 1];
//     front = 0;
//     rear = 0;
//     max_size = n + 1;
// }

bool circularQueue::init(const int& n)
{
    if (arr == 0)
        arr = new double[n + 1];
    front = 0;
    rear = 0;
    max_size = n + 1;  
    return 1;
}


bool circularQueue::quit(void)
{
    delete[] arr;
    return 1;
}


int circularQueue::length(void) const
{
    return (rear - front + max_size) % max_size;
}


bool circularQueue::isEmpty(void) const
{
    return rear == front;
}


bool circularQueue::isFull(void) const
{
    return (rear + 1) % max_size == front;
}


bool circularQueue::enqueue(const double& elem)
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


double circularQueue::dequeue(void)
{
    if (length() == 0)
    {
        double tmp;
        return tmp;
    }
    else
    {
        double tmp = arr[front];
        //arr[front] = 0;
        front = (front + 1) % max_size;
        return tmp;
    }
}


double& circularQueue::operator[](const int& n)
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


double& circularQueue::get(const int& n) const
{
    return arr[(rear + max_size - n) % max_size];
}


bool circularQueue::clear(void)//清空
{
    while (!isEmpty())
    {
        dequeue();
    }
    return 1;
}


bool circularQueue::remove(const int& n)//删掉几个
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
    pitch_result = 0;//这里用来存放最终的预测结果

    target.x = 0;
    target.y = 0;
    target.z = 0;
    target.xy_plane_distance = 0;
    target.yaw = 0;


    yaw_pre.quit();
    pitch_pre.quit();
    yaw_pre.init(queue_length);
    pitch_pre.init(queue_length);
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

    //cv::FileNode cam = fsread["camera_internal_matrix"]["data"];
    //cv::Mat e(3, 3, CV_64FC1);
    //printf("%d\n", cam);
    //cam[0] >> e;//失败的尝试1

    // cv::FileNode fn4 = fsread["camera_internal_matrix"];
    // cv::FileNode fn4l = fn4["data"];
    // std::cout << fn4l[0] << std::endl;
    // fn4l[0] >> cameraMatrix;//失败的尝试2

    //cameraMatrix.resize(rows_cam, cols_cam);
    //distCoeffs.resize(rows_dist, cols_dist);
    // fsread["camera_internal_matrix"]["data"] >> cameraMatrix;
    // fsread["distortion_coeff"]["data"] >> distCoeffs;//失败的尝试3

    //cv::Mat c(rows_cam, cols_cam, CV_32FC1, cv::Scalar::all(0));
    //fsread["camera_internal_matrix"]["data"] >> c;//失败的尝试4

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

    //std::cout << distCoeffs << std::endl;
    return 1;
}

// bool TargetSolver::receiveDta(void)
// {}

void TargetSolver::coordinateTrans(cv::Point3f inputPoint)//没理解错的话应该是Armor类调用我，即识别一个点转化一次
{
    std::vector<double> rvec;//旋转向量
    std::vector<double> tvec;//平移向量
    cv::Mat R;
    /*第一步：获取想要的参数*/
    //cv::RotatedRect Rect_1, Rect_2;//得改，因为这应该是从Armor里提取的
    std::vector<cv::Point2f> P2D;//从Armor里取4个点以及顺序
    P2D.push_back(cv::Point2f(140, 205));
    P2D.push_back(cv::Point2f(182, 204));
    P2D.push_back(cv::Point2f(182, 222));
    P2D.push_back(cv::Point2f(141, 223));

    // P2D.push_back(cv::Point2f(0.2, 0.2));
    // P2D.push_back(cv::Point2f(0.6, 0.2));
    // P2D.push_back(cv::Point2f(0.6, 0.4));
    // P2D.push_back(cv::Point2f(0.2, 0.4));

    bool isBig = 0;

    /*第二步：获取旋转向量以及平移向量*/
    if (0 == isBig)
    {
        cv::solvePnP(PW3D_Small, P2D, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    else
    {
        cv::solvePnP(PW3D_Big, P2D, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }


    std::cout << "tvec: ";
    for (auto& tmp : tvec)
    {
        std::cout << tmp << ' ';
    }
    std::cout << std::endl;

    /*第三步：旋转向量转化为旋转矩阵*/
    cv::Rodrigues(rvec, R);


    /*第四步：矩阵相乘，将输入点在世界系的坐标转化为在相机系的坐标*/
    cv::Mat P(4, 4, CV_64FC1);
    //cv::Mat x_input(4, 1, CV_64FC1);
    cv::Mat x_output(4, 1, CV_64FC1);


    x_output.at<double>(0, 0) = inputPoint.x;
    x_output.at<double>(1, 0) = inputPoint.y;
    x_output.at<double>(2, 0) = inputPoint.z;
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
    std::cout << "298: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << std::endl;
    /*第五步：矩阵相加并将坐标系绕x轴逆时针旋转90度，将目标点在相机系转为在云台系*/
    x_output.at<double>(2, 0) += length;//z的变换，length还需要和机械同学商量>
    x_output.at<double>(1, 0) -= height;//y的变换，height还需要和机械同学商量
    std::cout << "302: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << std::endl;

    double tmp = -x_output.at<double>(1, 0);
    x_output.at<double>(1, 0) = x_output.at<double>(2, 0);
    x_output.at<double>(2, 0) = tmp;//云台系1到云台系2

    std::cout << "308: x = " << x_output.at<double>(0, 0) << " y = " << x_output.at<double>(1, 0) << " z = " << x_output.at<double>(2, 0) << "yaw = " << 180 * atan2(x_output.at<double>(0,0), x_output.at<double>(1, 0)) / acos(-1.0) << std::endl;

    /*第六步：将该点在云台系的坐标转换为在大地坐标系的坐标*/
    
    double yaw = -5.5333 * acos(-1.0) / 180, pitch = 0.7388 * acos(-1.0) / 180, roll = -0.0277 * acos(-1.0) / 180;

    //get(yaw, pitch, roll);//这里需要通信那边的电控发给视觉的实时yaw，pitch，roll的值
    // cv::Mat YAW = (cv::Mat_<double>(4, 4) << cos(yaw), 0, sin(yaw), 0, 0, 1, 0, 0, -sin(yaw), 0, cos(yaw), 0, 0, 0, 0, 1);
    // cv::Mat PITCH = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, cos(pitch), -sin(pitch), 0, 0, sin(pitch), cos(pitch), 0, 0, 0, 0, 1);
    // cv::Mat ROLL = (cv::Mat_<double>(4, 4) << cos(roll), -sin(roll), 0, 0, sin(roll), cos(roll), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

    cv::Mat YAW = (cv::Mat_<double>(4, 4) << cos(yaw), -sin(yaw), 0, 0, sin(yaw), cos(yaw), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    cv::Mat PITCH = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, cos(pitch), -sin(pitch), 0, 0, sin(pitch), cos(pitch), 0, 0, 0, 0, 1);
    cv::Mat ROLL = (cv::Mat_<double>(4, 4) << cos(roll), 0, sin(roll), 0, 0, 1, 0, 0, -sin(roll), 0, cos(roll), 0, 0, 0, 0, 1);


    x_output = YAW * PITCH * ROLL * x_output;
    /*第七步：返回结果*/
    target.x = x_output.at<double>(0, 0);
    target.y = x_output.at<double>(1, 0);
    target.z = x_output.at<double>(2, 0);
    target.xy_plane_distance = sqrt(target.x * target.x + target.y * target.y);
    target.yaw = -atan2(target.y, target.x);
    /*这里还得考虑边缘情况*/
    /*测试区*/
    std::cout << "大地坐标系：";
    std::cout << "x = " << x_output.at<double>(0, 0) << ", y = " << x_output.at<double>(1, 0) << ", z = " << x_output.at<double>(2, 0) << std::endl;

    // std::cout << "rvec:" ;
    // for (auto& tmp : rvec)
    // {
    //     std::cout << tmp << " ";
    // }
    // std::cout << std::endl;
    // for (auto& tmp : tvec)
    // {
    //     std::cout << tmp << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "P:" << P << std::endl;

}

void TargetSolver::traceCal(void)
{
    double x = target.xy_plane_distance;
    double y = target.z;
    double tan_theta = 0.5;//初始值设置成0.5

    //方法一：不动点迭代，迭代5次
    // for (int i = 0; i < 5; ++i)
    // {
    //     tan_theta = (k_small * v0_small * y / (m_small * v0_small * (exp(k_small * x / m_small) - 1))) + m_small * 9.8 * (exp(k_small * x / m_small) - 1) * (1 + tan_theta * tan_theta) / (2 * k_small * v0_small * v0_small);
    // }

    //方法二：直接接二元一次方程
    tan_theta = (1 - sqrt(1 - 2 * 9.8 * y / (v0_small * v0_small))) * k_small * v0_small * v0_small / (m_small * 9.8 * (exp(k_small * x / m_small) - 1));//这里最开始的1 - 待定，可能是1 +

    t_hit = m_small * (exp(k_small * x / m_small) - 1) / (k_small * v0_small * cos(atan2(tan_theta, 1)));

    if (!yaw_pre.isFull())//如果队列未满（即初始化时）
    {
    }
    else
    {
        leastSquare();//先算一步
        yaw_pre.dequeue();
        pitch_pre.dequeue();
    }
    yaw_pre.enqueue(-atan2(target.x, target.y));
    pitch_pre.enqueue(atan2(tan_theta, 1));


}

void TargetSolver::leastSquare(void)//最小二乘求最优解
{
    double b_yaw = 0, a_yaw = 0, sigma_xy_yaw = 0, sigma_x_yaw = 0, sigma_y_yaw = 0, sigma_xx_yaw = 0;
    double b_pitch = 0, a_pitch = 0, sigma_xy_pitch = 0, sigma_x_pitch = 0, sigma_y_pitch = 0, sigma_xx_pitch = 0;

    for (int i = 0; i < queue_length; ++i)
    {
        sigma_xy_yaw += i * yaw_pre[i];
        sigma_x_yaw += i;
        sigma_y_yaw += yaw_pre[i];
        sigma_xx_yaw += i * i;

        sigma_xy_pitch += i * pitch_pre[i];
        sigma_x_pitch += i;
        sigma_y_pitch += pitch_pre[i];
        sigma_xx_pitch += i * i;

    }

    b_yaw = (sigma_xy_yaw - sigma_x_yaw * sigma_y_yaw / queue_length) / (sigma_xx_yaw - sigma_x_yaw * sigma_x_yaw / queue_length);
    a_yaw = (sigma_y_yaw - b_yaw * sigma_x_yaw) / queue_length;

    b_pitch = (sigma_xy_pitch - sigma_x_pitch * sigma_y_pitch / queue_length) / (sigma_xx_pitch - sigma_x_pitch * sigma_x_pitch / queue_length);
    a_pitch = (sigma_y_pitch - b_pitch * sigma_x_pitch) / queue_length;

    yaw_result = b_yaw * (queue_length + t_hit / delta_t + t_delay / delta_t) + a_yaw;
    pitch_result = b_pitch * (queue_length + t_hit / delta_t + t_delay / delta_t) + a_pitch;

}

void TargetSolver::test(void)
{
    double x = target.xy_plane_distance;
    double y = target.z;

    double tan_theta, pitch, yaw;

    //方法一：不动点迭代，迭代5次
    // for (int i = 0; i < 5; ++i)
    // {
    //     tan_theta = (k_small * v0_small * y / (m_small * v0_small * (exp(k_small * x / m_small) - 1))) + m_small * 9.8 * (exp(k_small * x / m_small) - 1) * (1 + tan_theta * tan_theta) / (2 * k_small * v0_small * v0_small);
    // }
    //方法二：直接接二元一次方程
    yaw = -180 * atan2(target.x, target.y) / acos(-1.0);
    tan_theta = (1 - sqrt(1 - 2 * 9.8 * y / (v0_small * v0_small))) * k_small * v0_small * v0_small / (m_small * 9.8 * (exp(k_small * x / m_small) - 1));//这里最开始的1 - 待定，可能是1 +
    pitch = 180 * atan2(tan_theta, 1) / acos(-1.0);
    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
}