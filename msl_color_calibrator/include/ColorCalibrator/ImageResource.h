#ifndef IMAGERESOURCE_H
#define IMAGERESOURCE_H

#include <map>
#include <string>

class ImageResource {

public:
    ImageResource(unsigned char * content, int width, int height);
    ~ImageResource();

    std::string m_filename;

    unsigned char getContentAt(int index);

    int getWidth() { return m_width; }
    int getHeight() { return m_height; }
    void setSender(int sender) { m_sender = sender; }
    int getSender() { return m_sender; }
    void setSenderIsFixed(bool fixedSender) { m_fixedSender = fixedSender; }
    bool isSenderFixed() { return m_fixedSender; }
private:
    int m_width;
    int m_height;

    unsigned char * m_content;

    int m_sender;
    bool m_fixedSender;
};

#endif // IMAGERESOURCE_H
