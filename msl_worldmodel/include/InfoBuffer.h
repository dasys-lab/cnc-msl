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
    friend class ::msl::RawSensorData;

  public:
    /**
     * Default constructor.
     * @param bufferSize Number of elements which can be stored within the information buffer
     */
    InfoBuffer(const int bufferSize)
    {
        this->bufferSize = bufferSize;
        this->infoElementCounter = 0;
        this->index = -1;
        this->ringBuffer = std::unique_ptr<InformationElement<T>[]>(new InformationElement<T>[this->bufferSize]);
    }

    /**
     *  Default destructor.
     */
    virtual ~InfoBuffer()
    {
    }

    /**
     * Return the buffer size.
     * @return the buffer size.
     */
    const int getBufferSize() const
    {
        return this->bufferSize;
    }

    /**
     * Returns the current number of elements within the information buffer.
     * @return the current number of elements within the information buffer.
     */
    const int getSize() const
    {
        return (this->bufferSize < this->infoElementCounter) ? this->bufferSize : this->infoElementCounter;
    }

    /**
     * Clears the buffer. If cleanBuffer is false only the index structure is reseted, but the
     * buffer still exists (old elements are not accessible). If cleanBuffer is true the
     * pointers from the buffer are cleared as well.
     * @param cleanBuffer True to clear the buffer.
     */
    void clear(bool cleanBuffer)
    {
        std::lock_guard<std::mutex> guard(mtx_);
        this->index = -1;
        this->infoElementCounter = 0;

        if (cleanBuffer)
        {
            InformationElement<T> element;
            for (int i = 0; i < this->bufferSize; ++i)
                this->ringBuffer[i] = element;
        }
    }

    /**
     * Returns the type_info of the used template type.
     * @return The type_info of the used template type.
     */
    const std::type_info *getTypeInfo() const
    {
        return &typeid(T);
    }

    /**
     * Sets the out parameter to the n-th last element, if it exists.
     * @param out This out parameter will be set if possible.
     * @param n States that the n-th last element should be set to the out parameter.
     * @return True if the n-th last element exists, false otherwise.
     */
    bool getLast(std::shared_ptr<InformationElement<T>> &out, const int n = 0) const
    {
        std::lock_guard<std::mutex> guard(mtx_);

        if (this->index < 0 || this->bufferSize <= n || this->infoElementCounter <= n)
        {
            return false;
        }

        out = this->ringBuffer[(this->index - n) % this->bufferSize];
        return true;
    }

    bool getLastValid(std::shared_ptr<InformationElement<T>> &out) const
    {
        std::lock_guard<std::mutex> guard(mtx_);

        if (this->index < 0 || this->bufferSize <= 0 || this->infoElementCounter <= 0)
        {
            return false;
        }

        int limit = std::min(this->bufferSize, this->infoElementCounter);
        for (int i = 0; i < limit; i++)
        {
            int index = (this->index - i) % this->bufferSize;
            if (this->ringBuffer[index].isValid())
            {
                out = this->ringBuffer[index];
                return true;
            }
        }
        return false;
    }

    bool getTemporalCloseTo(std::shared_ptr<InformationElement<T>> &out, const InfoTime time) const
    {
        std::lock_guard<std::mutex> guard(mtx_);

        if (this->index < 0 || this->bufferSize <= 0 || this->infoElementCounter <= 0)
        {
            return false;
        }

        InfoTime timeDiffOfClosest = std::numeric_limits<long long>::max();
        int numberOfAvailableElements = std::min(this->bufferSize, this->infoElementCounter);
        for (int i = 0; i < numberOfAvailableElements; i++)
        {
            int index = (this->index - i) % this->bufferSize;
            InfoTime curTimeDiff = std::abs(time - this->ringBuffer[index]->getCreationTime());
            if (curTimeDiff < timeDiffOfClosest)
            {
                out = this->ringBuffer[index];
                timeDiffOfClosest = curTimeDiff;
            }
            else
            {
                /*
                * We found the information closest to time in the last iteration,
                * because the ringBuffer is sorted.
                */
                break;
            }
        }

        return true;
    }

    bool getTemporalCloseTo(std::shared_ptr<InformationElement<T>> &out, const InfoTime time,
                            const InfoTime maxTimeDiff) const
    {
        if (!this->getTemporalCloseTo(out, time) || std::abs(time - out->getCreationTime()) > maxTimeDiff)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
         * Adds a new element to the information buffer, if it older than the last added element.
         * @param element The element to add.
         * @return bool True if the element was added, False otherwise.
         */
    bool add(std::shared_ptr<InformationElement<T>> element)
    {
        std::lock_guard<std::mutex> guard(mtx_);

        if (element->getCreationTime() < this->ringBuffer[this->index % this->bufferSize]->getCreationTime())
        {
            return false;
        }

        this->infoElementCounter++;
        this->index = (++this->index) % this->bufferSize;
        this->ringBuffer[index] = element;

        return true;
    }

  private:
    std::mutex mtx_;
    std::unique_ptr<std::shared_ptr<InformationElement<T>>[]> ringBuffer; /**< Ring buffer of elements */
    int bufferSize;                                                       /**< number of stored elements */
    int index;                             /**< Current index of the last added element */
    unsigned long long infoElementCounter; /**< Counter of elements added to the buffer */
};

} /* namespace msl */
