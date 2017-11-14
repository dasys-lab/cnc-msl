#include "msl_ptgrey_camera/MSLPtGreyCamera.h"

using namespace std;

namespace msl_ptgrey_camera
{

    MSLPtGreyCamera::MSLPtGreyCamera()
    {
        this->cam = nullptr;
        this->camImg = nullptr;
        this->cps = nullptr;
    }

    MSLPtGreyCamera::~MSLPtGreyCamera()
    {
        delete this->cam;
        delete this->camImg;
        delete this->cps;
    }

    void PrintError(FlyCapture2::Error error);

    void imgCallback(class FlyCapture2::Image *pImage, const void *pCallbackData)
    {
        cout << "callback!" << endl;
    }

    int MSLPtGreyCamera::init()
    {

        this->cam = new FlyCapture2::GigECamera();


        this->camImg = new FlyCapture2::Image();

        cps = new msl_ptgrey_camera::CamPropertySetter();

//		FlyCapture2::GigECamera cam;
        FlyCapture2::CameraInfo camInfo;
        FlyCapture2::Error error;

        FlyCapture2::BusManager busMgr;
        FlyCapture2::PGRGuid guid;
        //Format7PacketInfo fmt7PacketInfo;
        //Format7ImageSettings fmt7ImageSettings;


        FlyCapture2::TriggerMode triggerMode;
        // Get Flea2 camera

        busMgr.ForceAllIPAddressesAutomatically();

        error = busMgr.GetCameraFromIndex(0, &guid);

        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
        // Connect to the camera

        error = this->cam->Connect(&guid);

        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        // Get camera information
        error = cam->GetCameraInfo(&camInfo);

        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        // Set Config
        FlyCapture2::FC2Config config;
        config.grabMode = FlyCapture2::GrabMode::DROP_FRAMES;
        cam->SetConfiguration(&config);

        // Set Imaging Mode
        cam->SetGigEImagingMode(FlyCapture2::MODE_0);

        FlyCapture2::GigEImageSettings imageSettings;
        imageSettings.pixelFormat = FlyCapture2::PixelFormat::PIXEL_FORMAT_RAW8;
        imageSettings.height = this->imgHeight;
        imageSettings.width = this->imgWidth;
        cam->SetGigEImageSettings(&imageSettings);


        // Start Capture
        error = cam->StartCapture();
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
		cps->setCamera(this->cam);
		cps->setDefaults();

        return 0;

    }

    void MSLPtGreyCamera::setGamma(double value)
    {
        cps->setProperty(FlyCapture2::PropertyType::GAMMA, value);
    }

    void MSLPtGreyCamera::setShutter(double value)
    {
        cps->setProperty(FlyCapture2::PropertyType::SHUTTER, value);
    }

    void MSLPtGreyCamera::setGain(double value)
    {
        cps->setProperty(FlyCapture2::PropertyType::GAIN, value);
    }

    void MSLPtGreyCamera::setHue(double value)
    {
        cps->setProperty(FlyCapture2::PropertyType::HUE, value);
    }

    void MSLPtGreyCamera::setSaturation(double value)
    {
        cps->setProperty(FlyCapture2::PropertyType::SATURATION, value);
    }

    void MSLPtGreyCamera::setWhiteBalance(double redChannel, double blueChannel)
    {
        cps->setProperty(FlyCapture2::PropertyType::WHITE_BALANCE,redChannel, blueChannel);
    }

    void PrintError(FlyCapture2::Error error)
    {
        error.PrintErrorTrace();
    }

    unsigned char *MSLPtGreyCamera::getNextImage() const
    {
        cam->RetrieveBuffer(camImg);
        return camImg->GetData();
    }

    void MSLPtGreyCamera::saveCurrentImageToFile(string fpath)
    {
        this->cam->RetrieveBuffer(camImg);
        this->camImg->Save(fpath.c_str(), FlyCapture2::ImageFileFormat::RAW);
    }

    void MSLPtGreyCamera::initialiseParameters()
    {
        this->imgHeight = (*this->sc)["Vision2.conf"]->get<int>("Vision2.ScanLines.imgHeight", NULL);
        this->imgWidth = (*this->sc)["Vision2.conf"]->get<int>("Vision2.ScanLines.imgWidth", NULL);
    }


} /* namespace msl_ptgrey_camera */
