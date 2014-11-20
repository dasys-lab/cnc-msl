#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <time.h>

using namespace std;

class Robot {
public:
    Robot(int id, string hostname);
    ~Robot();

    unsigned char * m_lookupTable;

    int brightnessMin;
    int brightnessMax;
    int exposureMin;
    int exposureMax;
    int whiteBalance1Min;
    int whiteBalance1Max;
    int whiteBalance2Min;
    int whiteBalance2Max;
    int hueMin;
    int hueMax;
    int saturationMin;
    int saturationMax;
    int gammaMin;
    int gammaMax;
    int shutterMin;
    int shutterMax;
    int gainMin;
    int gainMax;

    int getID() { return m_ID; }
    string getHostname() { return m_hostname; }
    time_t getLastUpdate() { return m_lastUpdate; }

    bool usingBrightness() { return m_useBrightness; }
    void setUseBrightness(bool useBrightness) { setUseBrightness(useBrightness, true); }
    void setUseBrightness(bool useBrightness, bool update);

    int getBrightness() { return m_brightness; }
    void setBrightness(int brightness) { setBrightness(brightness, true); }
    void setBrightness(int brightness, bool update);

    int getExposure() { return m_exposure; }
    void setExposure(int exposure) { setExposure(exposure, true); }
    void setExposure(int exposure, bool update);

    bool isAutoWhiteBalanceUsed() { return m_autoWhiteBalance; }
    void setAutoWhiteBalance(bool autoWhiteBalance) { setAutoWhiteBalance(autoWhiteBalance, true); }
    void setAutoWhiteBalance(bool autoWhiteBalance, bool update);

    int getWhiteBalance1() { return m_whiteBalance1; }
    void setWhiteBalance1(int whiteBalance1) { setWhiteBalance1(whiteBalance1, true); }
    void setWhiteBalance1(int whiteBalance1, bool update);

    int getWhiteBalance2() { return m_whiteBalance2; }
    void setWhiteBalance2(int whiteBalance2) { setWhiteBalance2(whiteBalance2, true); }
    void setWhiteBalance2(int whiteBalance2, bool update);

    int getHue() { return m_hue; }
    void setHue(int hue) { setHue(hue, true); }
    void setHue(int hue, bool update);

    int getSaturation() { return m_saturation; }
    void setSaturation(int saturation) { setSaturation(saturation, true); }
    void setSaturation(int saturation, bool update);

    bool isGamma() { return m_enabledGamma; }
    void setEnabledGamma(bool enabledGamma) { setEnabledGamma(enabledGamma, true); }
    void setEnabledGamma(bool enabledGamma, bool update);

    int getGamma() { return m_gamma; }
    void setGamma(int gamma) { setGamma(gamma, true); }
    void setGamma(int gamma, bool update);

    bool isAutoShutterUsed() { return m_autoShutter; }
    void setAutoShutter(bool autoShutter) { setAutoShutter(autoShutter, true); }
    void setAutoShutter(bool autoShutter, bool update);

    int getShutter() { return m_shutter; }
    void setShutter(int shutter) { setShutter(shutter, true); }
    void setShutter(int shutter, bool update);

    bool isAutoGainUsed() { return m_autoGain; }
    void setAutoGain(bool autoGain) { setAutoGain(autoGain, true); }
    void setAutoGain(bool autoGain, bool update);

    int getGain() { return m_gain; }
    void setGain(int gain) { setGain(gain, true); }
    void setGain(int gain, bool update);
private:
    int m_ID;
    string m_hostname;

    bool m_useBrightness;
    int m_brightness;
    int m_exposure;
    bool m_autoWhiteBalance;
    int m_whiteBalance1;
    int m_whiteBalance2;
    int m_hue;
    int m_saturation;
    bool m_enabledGamma;
    int m_gamma;
    bool m_autoShutter;
    int m_shutter;
    bool m_autoGain;
    int m_gain;

    time_t m_lastUpdate;
};

#endif // ROBOT_H
