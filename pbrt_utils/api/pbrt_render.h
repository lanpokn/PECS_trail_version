#ifndef MY_PBRT_RENDER_H
#define MY_PBRT_RENDER_H
#include<api/render.h>

#ifdef PBRT_BUILD_GPU_RENDERER
#include <pbrt/gpu/memory.h>
#include <pbrt/gpu/util.h>
#endif  // PBRT_BUILD_GPU_RENDERER
#include <pbrt/options.h>
#include <pbrt/shapes.h>
#include <pbrt/util/check.h>
#include <pbrt/util/color.h>
#include <pbrt/util/colorspace.h>
#include <pbrt/util/display.h>
#include <pbrt/util/error.h>
#include <pbrt/util/memory.h>
#include <pbrt/util/parallel.h>
#include <pbrt/util/print.h>
#include <pbrt/util/spectrum.h>
#include <pbrt/util/stats.h>

#include <ImfThreading.h>

#include <stdlib.h>

#include <pbrt/pbrt.h>

#include <pbrt/cpu/render.h>
#ifdef PBRT_BUILD_GPU_RENDERER
#include <pbrt/gpu/memory.h>
#endif  // PBRT_BUILD_GPU_RENDERER
#include <pbrt/options.h>
#include <pbrt/parser.h>
#include <pbrt/scene.h>
#include <pbrt/util/args.h>
#include <pbrt/util/check.h>
#include <pbrt/util/error.h>
#include <pbrt/util/log.h>
#include <pbrt/util/memory.h>
#include <pbrt/util/parallel.h>
#include <pbrt/util/print.h>
#include <pbrt/util/spectrum.h>
#include <pbrt/util/string.h>
#include <pbrt/wavefront/wavefront.h>
#include <pbrt/util/image.h>


#include <ImfChannelList.h>
#include <ImfChromaticitiesAttribute.h>
#include <ImfFloatAttribute.h>
#include <ImfFrameBuffer.h>
#include <ImfHeader.h>
#include <ImfInputFile.h>
#include <ImfIntAttribute.h>
#include <ImfMatrixAttribute.h>
#include <ImfOutputFile.h>
#include <ImfStringAttribute.h>
#include <ImfStringVectorAttribute.h>

#include <string>
#include <vector>
#include <iostream>
using namespace pbrt;

//use it to memory the exr data
//use static to avoid multi defination
//use extern to fix this problem!
namespace pbrt_render_h{
// extern class Imf::FrameBuffer  EXRFrameBuffer;
extern Image image;
extern ImageChannelDesc desc;
extern Imath::Box2i dataWindow;
extern int resolutiony;
extern class Imf::Header header;
}

/**
 * @brief all from 0 to 1
 * 
 */
typedef struct apiBound_struct{
    float x0 = 0;
    float x1 = 1;
    float y0 = 0;
    float y1 = 1;
    apiBound_struct(float x0_in = 0,float x1_in = 1,float y0_in = 0,float y1_in = 1){
        x0 = x0_in;
        x1 = x1_in;
        y0 = y0_in;
        y1 = y1_in;
    }
private:
    bool changed = false;
} apiBound;
struct PbrtConfig:BasicConfig{
    //cmd 
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
    //changed
    int pixelSamples = -1;
    int gpuDevice =  -1;
    bool quickRender = false;
    // bool upgrade = false;
    // std::string imageFile;
    // std::string mseReferenceImage, mseReferenceOutput;
    // std::string debugStart;
    // std::string displayServer;
    //changed
    apiBound cropWindow;//changed by hhq
    // Bounds2f projRearBounds(Point2f(-1.5f * rearRadius, -1.5f * rearRadius),
    //                         Point2f(1.5f * rearRadius, 1.5f * rearRadius));
    // pstd::optional<Bounds2i> pixelBounds;
    // pstd::optional<Point2i> pixelMaterial;
    // Float displacementEdgeScale = 1;
    
    //it will be used when parsefile, if it is not null, we will create a new pbrt configfile(with a different name), && then delete it
    //because original file is very conplex && only use a filename as input, do so can avoid change code
    //TODO
    //vector<CheckInfo*> info;
    std::vector<RealisticCameraParam> RealCameraList;
    std::vector<PerspectiveCameraParam> PerspectiveCameraList;
    std::vector<SphericalCameraParam> SphericalCameraList;
    std::vector<OrthographicCameraParam> OrthographicCameraList;
    //output
    Image image;
    ImageChannelDesc desc;
    Imath::Box2i dataWindow;
    int resolutiony;
    Imf::Header header;
    Imf::FrameBuffer imageToFrameBuffer(const Image &image,
                                           const ImageChannelDesc &desc,
                                           const Imath::Box2i &dataWindow);
    void WriteExr(std::string name);
    void AddRealCamera(float shutteropen = 0 ,float shutterclose = 1,
                     std::string lensfile = "",float diagonal_input = 35,
                     std::string Look_at = "",
                     float aperturediameter = 1.0,
                       float focusdistance = 10.0, std::string aperture = "circular")
    { 
        std::string label_input = "RealCamera_"+std::to_string(RealCameraList.size());
        RealisticCameraParam t(shutteropen,shutterclose,label_input,Look_at,diagonal_input,lensfile,aperturediameter,focusdistance,aperture);
        RealCameraList.push_back(t);
    }
    void AddPerspectiveCamera(float shutteropen = 0 ,float shutterclose = 1,
                              std::string Look_at = "",
                              float frameaspectratio_input = -1,
                              float screenwindow_input  = -1.0,float lensradius_input = 1,
                              float focaldistance_input =-1,float fov_input=90)
    {
        std::string label_input = "PerspectiveCamera_"+std::to_string(PerspectiveCameraList.size());
        PerspectiveCameraParam t(shutteropen,shutterclose,label_input,Look_at,frameaspectratio_input,screenwindow_input,lensradius_input,focaldistance_input,fov_input);
        PerspectiveCameraList.push_back(t);
    }
    void AddSphericalCamera(float shutteropen_input = 0,float shutterclose_input = 1,
                    std::string Look_at = "",
                    std::string mapping_input = "equalarea")
    {
        std::string label_input = "SphericalCamera_"+std::to_string(SphericalCameraList.size());
        SphericalCameraParam t(shutteropen_input,shutterclose_input,Look_at,label_input,mapping_input);
        SphericalCameraList.push_back(t);
    }
    void AddOrthographicCamera(float shutteropen_input = 0,float shutterclose_input = 1,
                    std::string Look_at = "",
                    float frameaspectratio_input = -1,float screenwindow_input  = -1.0,float lensradius_input = 1,
                    float focaldistance_input = -1)
    {
        std::string label_input = "OrthographicCamera_"+std::to_string(OrthographicCameraList.size());
        OrthographicCameraParam t(shutteropen_input,shutterclose_input,Look_at,label_input,frameaspectratio_input,screenwindow_input,lensradius_input,focaldistance_input);
        OrthographicCameraList.push_back(t);
    }
    /**
     * @brief convert it to a cmd string
     * 
     * @return std::string 
     */
    virtual std::string ToString() const;
};
/**
 * @brief render an image with pbrt method
 * 
 */
class pbrt_render{
  private:
    std::string cmd_input = "";
    // //these params have to be add because they are not one of  command line
    // std::vector<RealisticCameraParam> RealCameraList;
    // std::vector<PerspectiveCameraParam> PerspectiveCameraList;
    // std::vector<SphericalCameraParam> SphericalCameraList;
    // std::vector<OrthographicCameraParam> OrthographicCameraList;
    /**
     * @brief copy from the pbrt,
     * 
     * @param argv the input of the cmd
     * @return int 
     */
    int pbrt_main(int argc, char *argv[]);
    /**
     * @brief generate a new .pbrt file && renturn the new filename
     * 
     * @param RC 
     * @param filenames 
     */
  public:
    std::vector<PbrtConfig> Configlist;
    bool is_run = false;
    // Image image;
    // ImageChannelDesc desc;
    // Imath::Box2i dataWindow;
    // int resolutiony;
    // Imf::Header header;
    /**
     * @brief change config to string, && then call init(str)
     * 
     * @param con the config edit by the user
     * @return true 
     * @return false 
     */
    virtual bool AddConfig(PbrtConfig& con);
    /**
     * @brief get a string from the user, then run() will change it to char *argv[] which can be understood by the pbrt_main
     *        can add multi time, but these settings will all use the first con(except scene path) because initPBRT can only use once
     * @param str the str should be the same as the cmd input, such as --display-server localhost:14158 scene/explosion/explosion.pbrt
     *            these should be space between words
     * @return true 
     * @return false 
     */
    virtual bool initWithCmd(std::string &str);
    /**
     * @brief just need to call the pbrt_main with current cmd_input(change to char*[])
     *        anytime before calling a run, there should be an init() before it
     *        TODO:change run() to avoid change cmd_vel when dealing with it
     * 
     * @return true 
     * @return false 
     */
    virtual bool initWithCmd(std::string &&str);
    virtual bool run();
};

#endif  //MY_PBRT_RENDER_H