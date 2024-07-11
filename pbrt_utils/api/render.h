#ifndef MY_RENDER_H
#define MY_RENDER_H
#include<iostream>
#include<vector>
#include<math.h>

//reference one config may have different ,we need to decompose!
/**
 * @brief config locate a lot of settings && give it to the render，note thta there is no virtual varible
 * 
 */
//config设计时要保证用户可以用最基本的操作对config进行设计，而不需要引入额外库，最简单的方法就是看这些pbrt自己定义的类的构造变量都用了哪些数据
struct BasicConfig{
    // std::vector<std::string> scene_path;
    std::string scene_path;
    // int seed = 0;
    bool quiet = false;
    // bool disablePixelJitter = false, disableWavelengthJitter = false;
    // bool disableTextureFiltering = false;
    // bool forceDiffuse = false;
    bool useGPU = false;
    bool wavefront = false;
    // bool interactive = false;
    // RenderingCoordinateSystem renderingSpace = RenderingCoordinateSystem::CameraWorld;
    
    int nThreads = 0;
    // LogLevel logLevel = LogLevel::Error;
    // std::string logFile;
    // bool logUtilization = false;
    // bool writePartialImages = false;
    // bool recordPixelStatistics = false;
    // bool printStatistics = false;
    // pstd::optional<int> pixelSamples;
    // pstd::optional<int> gpuDevice;
    bool quickRender = false;
    // bool upgrade = false;
    // std::string imageFile;
    // std::string mseReferenceImage, mseReferenceOutput;
    // std::string debugStart;
    // std::string displayServer;
    // pstd::optional<Bounds2f> cropWindow;
    // pstd::optional<Bounds2i> pixelBounds;
    // pstd::optional<Point2i> pixelMaterial;
    // Float displacementEdgeScale = 1;

    virtual std::string ToString() const = 0;
};
/**
 * @brief remember all param, label is to distingguish between different camera that user defined
 * 
 */
struct CameraParam{
    float shutteropen = 0;
    float shutterclose = 1;
    std::string label = "";
    std::string Look_at = "";
    CameraParam(float shutteropen_input = 0,float shutterclose_input = 1,std::string label_input = "",
                std::string Look_at_input = ""){
        shutteropen = shutteropen_input;
        shutterclose = shutterclose_input;
        label = label_input;
        Look_at = Look_at_input;
    }
};
struct RealisticCameraParam:CameraParam{
    std::string lensfile = "";//use relative path
    float diagonal = 35;
    float aperturediameter = 1.0;
    float focusdistance = 10.0;
    // Allows specifying the shape of the camera aperture, 
    // which is circular by default. The values of "gaussian", "square", "pentagon", && "
    // star" are associated with built-in aperture shapes; o
    // ther values are interpreted as filenames specifying an image to be used to specify the shape.
    std::string aperture = "circular";

    RealisticCameraParam(float shutteropen_input = 0,float shutterclose_input = 1,std::string label_input = "",
                        std::string Look_at_input = "",
                        float diagonal_input = 35,
                        std::string lensfile_input = "",float aperdia_input = 1.0,float focusd_input = 10.0,
                        std::string aperture_input = "circular")
                        :CameraParam(shutteropen_input,shutterclose_input,label_input,Look_at_input)
    {
        lensfile = lensfile_input;
        aperturediameter = aperdia_input;
        focusdistance = focusd_input;
        aperture = aperture_input;
        diagonal = diagonal_input;
    }
};
struct PerspectiveCameraParam:CameraParam{
    float frameaspectratio = -1;//no need to input 
    float screenwindow = -1;//no need to input 
    float lensradius = 1;
    float focaldistance = 30;
    float fov=90;
    PerspectiveCameraParam(float shutteropen_input = 0,float shutterclose_input = 1,std::string label_input = "",
                    std::string Look_at_input = "",
                    float frameaspectratio_input = -1,float screenwindow_input  = -1.0,float lensradius_input = 1,
                    float focaldistance_input = -1,float fov_input=90)
                    :CameraParam(shutteropen_input,shutterclose_input,label_input,Look_at_input)
    {
        frameaspectratio = frameaspectratio_input;
        screenwindow = screenwindow_input;
        lensradius = lensradius_input;
        focaldistance = focaldistance_input;
        fov = fov_input;
    }
};
struct SphericalCameraParam:CameraParam{
    std::string mapping = "equalarea";
      SphericalCameraParam(float shutteropen_input = 0,float shutterclose_input = 1,std::string label_input = "",
                    std::string Look_at_input = "",
                    std::string mapping_input = "equalarea")
                    :CameraParam(shutteropen_input,shutterclose_input,label_input,Look_at_input)
    {
        mapping = mapping_input;
    }
};
struct OrthographicCameraParam:CameraParam{
    float frameaspectratio;//no need to input 
    float screenwindow;//no need to input 
    float lensradius = 1;
    float focaldistance = 30;
    OrthographicCameraParam(float shutteropen_input = 0,float shutterclose_input = 1,std::string label_input = "",
                    std::string Look_at_input = "",
                    float frameaspectratio_input = -1,float screenwindow_input  = -1.0,float lensradius_input = 1,
                    float focaldistance_input = -1)
                    :CameraParam(shutteropen_input,shutterclose_input,label_input,Look_at_input)
    {
        frameaspectratio = frameaspectratio_input;
        screenwindow = screenwindow_input;
        lensradius = lensradius_input;
        focaldistance = focaldistance_input;
    }
};
// this class has no meaning at all
// /**
//  * @brief totaly virtual class, won't have a instance
//  * 
//  */
// class render{
//   public:
//     // /**
//     //  * @brief set all the parameter with a config struct
//     //  * 
//     //  * @param con  set by the users
//     //  * @return true 
//     //  * @return false 
//     //  */
//     // virtual bool init(BasicConfig &con) = 0;
//     /**
//      * @brief set all the parameter with a string, such as a cmd in pbrt
//      * 
//      * @param str 
//      * @return true 
//      * @return false 
//      */
//     virtual bool init(std::string &str) = 0;
//     /**
//      * @brief almost the same with upper one
//      * 
//      * @param str 
//      * @return true 
//      * @return false 
//      */
//     virtual bool init(std::string &&str) = 0;
//     /**
//      * @brief after init, use this function to render a picture
//      * 
//      * @return true 
//      * @return false 
//      */
//     virtual bool run() = 0;
  
// };

#endif