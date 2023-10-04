#include "../include/uart.hpp"
#include "../include/coordinateTrans.hpp"
#include "../include/Track.hpp"
#include "../include/CamDrv.hpp"

// 处理线程
void ReceiveThread(my_data &data, uart &uart1)
{
    while (true)
    {
        uart1.my_read(data);
        if (data.read_pack())
        {
            // printf("%f\t%f\n")
            // printf("%ld\t%ld\n",data.ts.GetTimeStamp().time_ms,data.base_mcu_time.time_ms);
        }
    }
}
void SendThread(my_data &data, uart &uart1)
{
    while (true)
    {
        // printf("entre\n");
        // pose_pack pose;
        unsigned long long delta_time = data.ts.GetTimeStamp().time_us;
        // uint8_t tep[10] = {0x00};
        // write(uart1.fd, tep, 10);
        if (data.sendFlagPack.target_found)
        {
            if (data.write_pack(data.tep))
            {
                uart1.my_write(data);
            }
        }
        else
        {
            pose_pack tep;
            if(data.write_pack(tep))
            uart1.my_write(data);
        }
        delta_time = data.ts.GetTimeStamp().time_us - delta_time;
        // printf("%ld\n",delta_time);
        // printf("%ld\n",data.ts.GetTimeStamp().time_ms);
        if (delta_time < 20000)
        {
            usleep(20000 - delta_time);
        }
        else
        {
            usleep(20000);
        }
    }
}
int main()
{
    uart uart1;
    uart1.uart_init();
    my_data test;
    thread t1(ReceiveThread, ref(test), ref(uart1));
    thread t2(SendThread, ref(test), ref(uart1));
    TargetSolver ts;
    ts.readParas("../config/camera_param2.xml");
    // ts.coordinateTrans(cv::Point3f(0, 0, 0));
    // ts.traceCal();
    double score_max = 0;
    int i_max = 0;
    PreProcessing preprocess;
    Track tracking;
    MVCamera *c = new MVCamera();
    std::string path = "../config/camera.yaml";
    c->load_param(path); // 加载参数
    c->open();           // 打开摄像头
    while (true)
    {
        Mat frame;
        c->get_Mat(frame);
        cv::imshow("src", frame);
        cv::waitKey(5);
        // 取图
        // cv::imshow("test",frame);
        my_time mt;
        vector<vector<Point>> cs = preprocess.PreProcess(frame, 57, 94, 255, 230);
        vector<LightBar> lightbars = preprocess.Contours2LightBar(cs);
        vector<Armor> Armors = preprocess.getArmor(lightbars);
        for (int i = 0; i < Armors.size(); i++)
        {
            Armors[i].get_number_Image(frame);
            Armors[i].get_number();
        }
        vector<Armor> FliteredArmors = preprocess.ArmorFliter(Armors); // 将输入的装甲板类中不能被模型识别的部分踢掉

        if (FliteredArmors.size() != 0)
        {
            for (int i = 0; i < FliteredArmors.size(); ++i)
            {
                if (FliteredArmors[i].Score() > score_max)
                {
                    score_max = FliteredArmors[i].Score();
                    i_max = i;
                }
            }
            test.sendFlagPack.target_found = true;
            // std::cout <<"FliteredArmors[i_max].getPoints()"<< FliteredArmors[i_max].getPoints() << std::endl;
            // test.pose_pack_queue.front()
            test.tep = ts.coordinateTrans(cv::Point3f(0, 0, 0), FliteredArmors[i_max].getPoints(), mt, test);
            std::cout<<"pitch:"<<test.tep.ptz_pitch<<endl<<"yaw:"<<test.tep.ptz_yaw<<endl;
            // std::cout<<"end!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }
        else
        {
            test.sendFlagPack.target_found = false;
        }
    }

    t1.join();
    t2.join();

    return 0;
}