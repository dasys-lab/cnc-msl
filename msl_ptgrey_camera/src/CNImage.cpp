#include "msl_ptgrey_camera/CNImage.h"

#include <iostream>
#include <cstring>

using std::array;

using namespace std;
namespace msl_ptgrey_camera
{

    CNImage::CNImage()
    {
    }

    CNImage::CNImage(CNImage &origin)
    {

        this->height = origin.height;
        this->width = origin.width;
        this->channels = origin.channels;
        this->step = origin.width * origin.channels;
        this->imgData = (unsigned char *) malloc(origin.channels * origin.height * origin.width);
        memcpy(this->imgData, origin.imgData,
               origin.width * origin.height * origin.channels);

    }


    CNImage::CNImage(int height, int width,  int channels)
    {
        this->height = height;
        this->width = width;
        this->channels = channels;
        this->step = channels * width;
//        this->imgData = (unsigned char *) malloc(channels * height * width);
        this->imgData = nullptr;
    }

    CNImage CNImage::conv2d(array<array<int, 3>, 3> kernel)
    {

        CNImage output = *this;

        int sum;
        int scale = 1;


        //foreach pixel in picture
        for (int i = kernel.size() / 2; i < output.height - kernel.size() / 2; ++i)
        {
            for (int j = 3 / 2; j < output.width - kernel.size() / 2; ++j)
            {

                //set accumulated sum of current pixel to 0
                sum = 0;

//				for (int k = -(int)kernel.size() / 2; k <= kernel.size() / 2; ++k)
                for (int k = -1; k <= 1; ++k)
                {
                    for (int l = -1; l <= 1; ++l)
                    {

                        int data = output.imgData[(i - l) * output.step + j - k];
                        int co = kernel[k + kernel.size() / 2][l + kernel.size() / 2];
                        sum += data * co;

                    }
                }
                output.imgData[j + i * output.step] = sum / scale;
            }
        }

        return output;

    }

    void CNImage::conv1dH(CNImage &input, std::array<int, 3> kernel)
    {
        int scale;
        for (int elem : kernel) {
            scale+=elem;
        }

        for (int i = 0; i < input.height * input.width; i++)
        {
            int sum = 0;
            for (int j = 0; j < kernel.size(); j++)
            {
                sum += input.imgData[i - j] * kernel[j];
                input.imgData[i] = sum / scale;
            }
        }
    }

    void CNImage::conv1dV(CNImage &input, std::array<int, 3> kernel)
    {
        int colIdx = 0;
        for (int i = 0; i < input.height * input.width; i += input.step)
        {
            int sum = 0;
            for (int j = 0; j < kernel.size(); j++)
            {
                sum += input.imgData[i - j] * kernel[j];
                input.imgData[i] = sum / 4;
            }
            if (i + input.step >= input.height * input.width - 1)
            {
                i = colIdx;
                colIdx++;
            }
        }
    }


    CNImage::~CNImage()
    {
        free(imgData);
    }

} // namespace msl_ptgrey_camera
