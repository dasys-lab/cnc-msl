#include "Robot.h"

#include <cstdlib>

Robot::Robot(int id, string hostname) :
    m_ID(id),
    m_hostname(hostname)
{
    m_lookupTable = (unsigned char *) malloc(256*256 + 2);

    m_lastUpdate = 0;
}

Robot::~Robot() {
    free(m_lookupTable);
}

void Robot::setUseBrightness(bool useBrightness, bool update) {
    m_useBrightness = useBrightness;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setBrightness(int brightness, bool update) {
    m_brightness = brightness;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setExposure(int exposure, bool update) {
    m_exposure = exposure;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setAutoWhiteBalance(bool autoWhiteBalance, bool update) {
    m_autoWhiteBalance = autoWhiteBalance;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setWhiteBalance1(int whiteBalance1, bool update) {
    m_whiteBalance1 = whiteBalance1;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setWhiteBalance2(int whiteBalance2, bool update) {
    m_whiteBalance2 = whiteBalance2;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setHue(int hue, bool update) {
    m_hue = hue;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setSaturation(int saturation, bool update) {
    m_saturation = saturation;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setEnabledGamma(bool enabledGamma, bool update) {
    m_enabledGamma = enabledGamma;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setGamma(int gamma, bool update) {
    m_gamma = gamma;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setAutoShutter(bool autoShutter, bool update) {
    m_autoShutter = autoShutter;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setShutter(int shutter, bool update) {
    m_shutter = shutter;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setAutoGain(bool autoGain, bool update) {
    m_autoGain = autoGain;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}

void Robot::setGain(int gain, bool update) {
    m_gain = gain;
    if (update) {
        m_lastUpdate = time(NULL);
    }
}
