

#ifndef CAMDRV_HPP_
#define CAMDRV_HPP_

#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>

class MVCamera {
private:
	CameraHandle pCameraHandle;
	tSdkCameraDevInfo *pCameraInfo;
	unsigned char* m_pFrameBuffer;
	int CamNum;
	int frame_width;
	int frame_height;
	tSdkCameraCapbility tCapability;
	/// 曝光时间, 主要是用来输出
    double exposure_time;

    /// 帧率,<br>0: low <br> 1: normal <br> 2: high
    int current_frame_speed_index = 2;

    /// B通道增益，默认100
    int blue_channel_gain = 100;

    /// G通道增益，默认100
    int green_channel_gain = 130;

    /// R通道增益，默认100
    int red_channel_gain = 160;

    /// 饱和度，默认100
    int saturation = 100;

    /// 对比度，默认100
    int contrast = 100;

    /// 伽马值，默认100
    int gamma = 100;

    /// 设置图像锐化程度[0-100],默认0
    int sharpness = 0;

    /*******设置手动曝光相关参数************************/
    /// 自瞄曝光时间，需要动态调节,注意是double这个数据类型
    double armor_exposure_time = 3000;

    /// 能量机关曝光时间
    double rune_exposure_time = 1500;

    /// 打击哨兵的曝光
    double sentinel_exposure_time = 1000;

    /// 模拟增益
    int analog_gain = 64;
    /*****************END************************************/

    /****** 设置自动曝光相关参数**********************/
    /// 自动曝光目标亮度值, int
    int auto_exp_target_brightness = 55;

    /// 自动曝光阈值, int
    int auto_exp_thresh = 5;

    /// 自动曝光 曝光时间最小值, double
    double auto_exp_min_expourse_time = 100;

    /// 自动曝光 曝光时间最大值, double
    double auto_exp_max_expourse_time = 8000;

    /// 模拟增益默认设置为最高
    /// 自动曝光的模拟增益最小值, int
    int auto_exp_min_analog_gain = 128;

    /// 自动曝光的模拟增益最大值, int
    int auto_exp_max_analog_gain = 128;
    /*************END*************************/

public:
	MVCamera() {
		CameraSdkInit(0);
		CamNum = 1;
		m_pFrameBuffer=NULL;
		pCameraHandle = -1;
		pCameraInfo = (tSdkCameraDevInfo*) malloc(sizeof(tSdkCameraDevInfo));
		frame_height=480;
		frame_width=640;
	}
	~MVCamera() {
		CameraUnInit(pCameraHandle);
		delete pCameraInfo;
	}
	void static GrabImageCallback(CameraHandle, BYTE*, tSdkFrameHead*, PVOID);
	bool open();
	void get_Mat(cv::Mat&);

};

#endif /* CAMDRV_HPP_ */
