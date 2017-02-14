#pragma once

#include <memory>

namespace msl
{
typedef unsigned long long InfoTime;

/**
 * Information element stores one information and provides required meta
 * data. The information should not be changed.
 */
template <typename T>
class InformationElement
{
  public:
    InformationElement(std::shared_ptr<T> information, InfoTime creationTime, InfoTime validityTime, double certainty);
    InformationElement();
    virtual ~InformationElement();

    std::shared_ptr<T> getInformation() const;
    InfoTime getCreationTime() const;
    InfoTime getValidityTime() const;
    double getCertainty() const;
    bool isValid() const;

  private:
    std::shared_ptr<T> information; /**< the stored information */
    InfoTime creationTime;          /**< reception time of the information */
    InfoTime validityTime;          /**< the latest time this information is considered to be valid */
    double certainty;               /**< how certain the information was at the moment it was created */
    bool isDummy;
};

} /* namespace msl */
