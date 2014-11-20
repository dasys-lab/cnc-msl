#include "ImageResource.h"

#include <string.h>
#include <stdlib.h>

ImageResource::ImageResource(unsigned char * content, int width, int height) {
    m_width = width;
    m_height = height;
    m_content = (unsigned char *) malloc(width * height * 2);
    memcpy(m_content, content, width * height * 2);
    m_sender = -1;
    m_fixedSender = false;
}

ImageResource::~ImageResource() {
    free(m_content);
}

unsigned char ImageResource::getContentAt(int index) {
    return m_content[index];
}
