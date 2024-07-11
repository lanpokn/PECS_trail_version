#include<event/event.h>
#include<vector>
void test_cv(){
    cv::Mat img = cv::imread("/home/lanpokn/Documents/2022/pbrt/pbrt-v4/pbrt-v4/images/teaser-transparent-machines.png");
	cv::imshow("test", img);
	cv::waitKey(0);
	return;
}

void GenerateIntensityVideo(pbrt_render render, int row, int column, std::string name){
	cv::VideoWriter writer;
 
    //单通道
    //#define CV_32F  5 不知道哪里撞名字了
    cv::Mat img(render.Configlist.at(0).image.Resolution().x, render.Configlist.at(0).image.Resolution().y, CV_32F, cv::Scalar(128,0,0));// 参数(int rows, int cols, int type, const Scalar& s)
    // cout << "MM = " << endl << " " << MM << endl;
 
	// writer.open("intensity1.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, img.size(), false);
 
	cv::namedWindow("intensity", cv::WINDOW_NORMAL);
	cv::namedWindow("intensity", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("intensity", 20, 20);
	cv::resizeWindow("intensity", 512, 512);

    render.run();

	for (int j = 0; j < render.Configlist.size(); j++)
	{
        //load with pbrt_render
		//img = imread(filename);
        std::cout << "good";
        ImageChannelDesc intensityDesc = render.Configlist.at(j).image.GetChannelDesc({"intensity"});
        if(!intensityDesc){
            std::cout<<"intensity not found"<<std::endl;
        }
        for (int y = 0; y < render.Configlist.at(j).image.Resolution().y; ++y){
            for (int x = 0; x < render.Configlist.at(j).image.Resolution().x; ++x) {
                float pixelValue = render.Configlist.at(j).image.GetChannels({x, y}, intensityDesc).Average();
                img.at<float>(y, x) = pixelValue;
                // mono.SetChannel({x, y}, 0, avg);
            }
        }
    // for (int y = 0; y < mono.Resolution().y; ++y)
    //     for (int x = 0; x < mono.Resolution().x; ++x) {
    //         Float avg = apertureImage.GetChannels({x, y}, rgbDesc).Average();
    //         mono.SetChannel({x, y}, 0, avg);
    //     }
        std::cout << "good";
        cv::imshow("intensity", img);
        cv::waitKey();
        cv::imwrite(name+ std::to_string(j+120), img);
		// writer.write(img);
	}/**/
	return;
}