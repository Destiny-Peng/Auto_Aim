#include <coordinateTrans.hpp>
#include <Armor.h>
#include <LightBar.h>
#include <PreProcess.h>
#include <Track.h>

int main(void)
{
   TargetSolver ts;
   ts.readParas("./camera_param5.xml");
   //ts.coordinateTrans(cv::Point3f(0, 0, 0));
   //ts.traceCal();

    double score_max = 0;
    int i_max = 0;


    int time = 0;
    PreProcessing preprocess;
    Track tracking;
    VideoCapture cap("333.mp4");
    if (!cap.isOpened()) {
        cerr << "error" << endl;
        return -1;
    }
    Mat frame;
    start:
        time ++;
        cap >> frame;
        cout << "time:"<<time << endl;
        Mat showframe = frame.clone();
        Mat testframe = frame.clone();
        vector<vector<Point>> cs = preprocess.PreProcess(frame,57,94,255,230);
        vector<LightBar> lightbars = preprocess.Contours2LightBar(cs);
        vector<Armor> Armors = preprocess.getArmor(lightbars);
        for (int i = 0; i < Armors.size(); i++) {
            Armors[i].get_number_Image(frame);
            Armors[i].get_number();
        }
        vector<Armor> FliteredArmors = preprocess.ArmorFliter(Armors);//将输入的装甲板类中不能被模型识别的部分踢掉

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


            // for (auto tmp : FliteredArmors[i_max].getPoints())
            // {
            //     std::cout << tmp << std::endl;
            // }
            std::cout << FliteredArmors[i_max].getPoints() << std::endl;

            ts.coordinateTrans(cv::Point3f(0, 0, 0), FliteredArmors[i_max].getPoints());
        }


        preprocess.drawArmor(testframe, FliteredArmors);//绘制装甲板，对于有特殊要求的装甲板会绘制其目标编号（左上）和装甲板类型（右上）
        imshow("frame",testframe);
        tracking.track(FliteredArmors,time,30,100);//第二个参数为目标失踪帧数容忍度
        //,148,202,204,236
        preprocess.drawLightBar(showframe, lightbars);
        preprocess.drawArmor(showframe, FliteredArmors);
        imshow("Processed", showframe);
        

        int key = waitKey(0);
        if (key == ' ') {
            goto start;
        }

    cap.release();
    destroyAllWindows();
   return 0;
}
