#ifndef CAMERA_FRAME_H
#define CAMERA_FRAME_H

#ifndef __orogen
#include <stdint.h>
#include <memory.h>

#include <vector>
#include <string>
#include <iostream>
#endif

//#include <dfki/base_types.h>

namespace camera
//namespace castor

{

    struct frame_size_t {
#ifndef __orogen
        frame_size_t() : width(0), height(0) {}
        frame_size_t(uint16_t w, uint16_t h) : width(w), height(h) {}
#endif

        uint16_t width;
        uint16_t height;
    };

    enum frame_mode_t {
        MODE_UNDEFINED = 0,
        MODE_RAW = 1,
        MODE_HDR = 2,
        MODE_GREYSCALE = 3, MODE_GRAYSCALE = 3,
        MODE_COLOUR = 4,    MODE_COLOR = 4,
        MODE_YUV422 = 5
    };

#ifndef __orogen
    template<int pixel_size> struct PixelTraits;
    template<>
    struct PixelTraits<8>
    {
        typedef uint8_t channel_t;
    };

    template<>
    struct PixelTraits<16>
    {
        typedef uint16_t channel_t;
    };
#endif

    /* A single image frame */
    struct Frame
    {
#ifndef __orogen
        public:
            /**
             * Initialize the frame
             * @param width the image width, in pixels
             * @param height the image height, in pixels
             * @param data_depth the number of effective bits per pixels
             * @param mode the frame mode (raw, hdr, greyscale, colour)
             */
            Frame() :
                stamp(0), image(), size(), data_depth(0), pixel_size(0),
                frame_mode()
            {
                setDataDepth(0);
                reset();
            }

            void init(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode)
            {
                this->frame_mode = mode;
                this->size = frame_size_t(width, height);

                setDataDepth(depth);

                image.resize(getPixelSize() * getPixelCount());
                reset();
//std::cout << "FRAME W: "<< width << " H: "<< height << " depth: " << depth << " mode: " << mode <<  " imgsize: " << getPixelSize() << " x " << getPixelCount() << std::endl;
            }

            void reset()
            {
                //this->stamp = DFKI::Time(0);
		this->stamp = 0;
                if (this->image.size() > 0) {
                    memset(&this->image[0], 0, this->image.size());
                }
            }

            bool isRaw()       { return this->frame_mode == MODE_RAW; }
            bool isHDR()       { return this->frame_mode == MODE_HDR; }
            bool isGrayscale() { return this->frame_mode == MODE_GRAYSCALE; }
            bool isColour()    { return this->frame_mode == MODE_COLOUR || this->frame_mode==MODE_YUV422; 
	    }

            int getChannelCount() const { return getChannelCount(this->frame_mode); }
            static int getChannelCount(frame_mode_t mode)
            {
                switch(mode)
                {
                    case MODE_RAW:       return 1;
                    case MODE_HDR:       return 1;
                    case MODE_GRAYSCALE: return 1;
		    case MODE_YUV422:	 return 2;
                    case MODE_COLOR:     return 3;
                    default:             return 0;
                }
            }


            inline frame_mode_t getFrameMode() const { return this->frame_mode; }

            /**
             * Returns the size of a pixel (in bytes). This takes into account the image
             * mode as well as the data depth.
             * @return Number of channels * bytes used to represent one colour
             */
            inline int getPixelSize() const { return this->pixel_size; }

            /**
             * Returns the total count of pixels in this frame
             * @return Returns the overall number of pixels (width * height)
             */
            inline uint32_t getPixelCount() const { return size.width * size.height; }

            inline uint32_t getDataDepth() const { return this->data_depth; }
            void setDataDepth(uint32_t value)
            {
                this->data_depth = value;

                // Update pixel size
                uint32_t comp_size = ((this->data_depth + 7) / 8);
                this->pixel_size = getChannelCount(this->frame_mode) * comp_size;
            }

            inline frame_size_t getSize() const { return this->size; }
            inline uint16_t getWidth() const { return this->size.width; }
            inline uint16_t getHeight() const { return this->size.height; }

            inline const std::vector<uint8_t> &getImage() const { return this->image; }

            inline void setImage(const std::vector<uint8_t> &image) { this->image = image; }
            inline void setImage(const char *data, uint32_t size) {
                if (size != this->image.size())
                {
                    std::cerr << "Frame: "
                        << __FUNCTION__ << " (" << __FILE__ << ", line "
                        << __LINE__ << "): " << "image size mismatch in setImage() ("
                        << size << " != " << this->image.size() << ")"
                        << std::endl;
                    return;
                }
		//this->image.resize(4*640*480*2);
                memcpy(&this->image[0], data, size);
		//unsigned char* speicherloch = new unsigned char[4*640*480*2];
		//unsigned char speicherloch[4*640*480*2];
		//if(speicherloch==NULL) std::cout << "SCHEISSE" << std::endl;
		//for(int i=0; i<size;i++) {
		//if (i>600000) printf("SCH %0x \n",&(data[i]));
		//	speicherloch[i] = data[i];
		//}
		//memcpy(&this->image[0], speicherloch, size/2);
            }

            inline uint8_t *getImagePtr() { return static_cast<uint8_t *>(&this->image[0]); }
            inline const uint8_t *getImageConstPtr() const { return static_cast<const uint8_t *>(&this->image[0]); }
#endif

            // The unix time at which the camFrame was captured
            // in the video1394 ringbuffer
            //DFKI::Time              stamp;
		long              stamp;
            std::vector<uint8_t>    image;

            // The image size [width, height]
            frame_size_t            size;

            // The number of effective bits per pixel. The number
            // of actual bits per pixels is always a multiple of
            // height (i.e. a 12-bit effective depth is represented
            // using 16-bits per channels). The number of greyscale
            // levels is 2^(this_number)
            uint32_t                data_depth;
            uint32_t                pixel_size;

            frame_mode_t            frame_mode;
    };
}

#endif

