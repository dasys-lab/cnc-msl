#include "Recorder.h"

using std::endl;
using std::cout;

Recorder::Recorder()
{
    this->sc = supplementary::SystemConfig::getInstance();
}

Recorder::~Recorder()
{}


void Recorder::initialiseParameters()
{
    this->maxImages = (*this->sc)["Vision2"]->get<int>("Vision2.Recorder.maxImages", NULL);
    this->freq = (*this->sc)["Vision2"]->get<int>("Vision2.Recorder.frequency", NULL);
    this->path = (*this->sc)["Vision2"]->get<string>("Vision2.Recorder.path", NULL);

}

int main(int argc, char **argv)
{

    Recorder rec;
    rec.initialiseParameters();

    msl_ptgrey_camera::MSLPtGreyCamera cam;
    auto error = cam.init();

    if(error == -1) {
        cout << "Error! Try again!" << endl;
        return error;
    }

    int frame = 0;
    while (frame < rec.maxImages)
    {
        frame++;
        string fpath = rec.path;
        fpath.append(std::to_string(frame));
        fpath.append(".raw");
        cout << fpath << endl;
//        cout << "frame:" << frame << ", maxImages: " << rec.maxImages << endl;
        cam.saveCurrentImageToFile(fpath);
        sleep(static_cast<unsigned int>(1 / std::max(0.033, (double)rec.freq)));
    }
    cout << "finished recording" << endl;
    return 0;
}
