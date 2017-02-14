#pragma once

#include "InformationElement.h"

#include <memory>
#include <mutex>
#include <typeinfo>


namespace msl
{
/**
 * Information buffer of shared_ptr of a given data type.
 */
template <typename T>
class InfoBuffer
{
  public:
    InfoBuffer(const int bufferSize);
    virtual ~InfoBuffer();

    const int getBufferSize() const;
    const int getSize() const;
    void clear(bool cleanBuffer);
    const std::type_info *getTypeInfo() const;

    int add(InformationElement<T> element);

    InformationElement<T> getLast(const int n = 0) const;
    InformationElement<T> getLastValid() const;

    InformationElement<T> getTemporalClose(const InfoTime time) const;
    InformationElement<T> getTemporalClose(const InfoTime time, const InfoTime timeInterval) const;

  private:
    std::mutex mtx_;
    std::unique_ptr<InformationElement<T>[]> ringBuffer; /**< Ring buffer of elements */
    int bufferSize;                                   /**< number of stored elements */
    int index;                                        /**< Current index of the last added element */
    unsigned long long infoElementCounter;            /**< Counter of elements added to the buffer */
};

} /* namespace msl */


