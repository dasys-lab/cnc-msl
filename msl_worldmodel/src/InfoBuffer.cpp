#include "InfoBuffer.h"

namespace msl
{

/**
 * Default constructor.
 * @param bufferSize Number of elements which can be stored within the information buffer
 */
template <typename T>
InfoBuffer<T>::InfoBuffer(const int bufferSize)
{
    this->bufferSize = bufferSize;
    this->infoElementCounter = 0;
    this->index = -1;
    this->ringBuffer = std::unique_ptr<InformationElement<T>[]>(new InformationElement<T>[this->bufferSize]);
}

/**
 *  Default destructor.
 */
template <typename T>
InfoBuffer<T>::~InfoBuffer()
{
}

/**
 * Return the buffer size.
 * @return the buffer size.
 */
template <typename T>
const int InfoBuffer<T>::getBufferSize() const
{
    return this->bufferSize;
}

/**
 * Returns the current count of elements within the information buffer.
 * @return the current count of elements within the information buffer.
 */
template <typename T>
const int InfoBuffer<T>::getSize() const
{
    return (this->bufferSize < this->infoElementCounter) ? this->bufferSize : this->infoElementCounter;
}

/**
 * Clears the buffer. If cleanBuffer is false only the index structure is reseted, but the
 * buffer still exists (old elements are not accessible). If cleanBuffer is true the
 * pointers from the buffer are cleared as well.
 * @param cleanBuffer True to clear the buffer.
 */
template <typename T>
void InfoBuffer<T>::clear(bool cleanBuffer)
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
template <typename T>
const std::type_info *InfoBuffer<T>::getTypeInfo() const
{
    return &typeid(T);
}

/**
 * Adds a new element to the information buffer and returns the identifier.
 * @param element The element to add.
 * @return the number of elements that have been added to the buffer
 */
template <typename T>
int InfoBuffer<T>::add(InformationElement<T> element)
{
    std::lock_guard<std::mutex> guard(mtx_);

    this->index = (++this->index) % this->bufferSize;

    this->ringBuffer[index] = element;

    return this->infoElementCounter++;
}

/**
 * Returns the n-th last element. Returns NULL if no element exists or n > maxBuffer.
 * @param n The n-th last element.
 * @return The n-th last element, 0 will return the newest one.
 */
template <typename T>
InformationElement<T> InfoBuffer<T>::getLast(const int n = 0) const
{
    std::lock_guard<std::mutex> guard(mtx_);

    if (this->index < 0 || this->bufferSize <= n || this->infoElementCounter <= n)
    {
        return InformationElement<T>();
    }

    int index = (this->index - n) % this->bufferSize;

    return this->ringBuffer[index];
}

template <typename T>
InformationElement<T> msl::InfoBuffer<T>::getLastValid() const
{
    std::lock_guard<std::mutex> guard(mtx_);

    if (this->index < 0 || this->bufferSize <= 0 || this->infoElementCounter <= 0)
    {
        return InformationElement<T>();
    }

    int n = 0;

    while (n < this->bufferSize)
    {
        int index = (this->index - n) % this->bufferSize;
        if (this->ringBuffer[index].isValid())
        {
            return this->ringBuffer[index];
        }
        else
        {
            n++;
        }
    }
    return InformationElement<T>();
}

template <typename T>
InformationElement<T> msl::InfoBuffer<T>::getTemporalClose(const InfoTime time) const
{
}

template <typename T>
InformationElement<T> msl::InfoBuffer<T>::getTemporalClose(const InfoTime time, const InfoTime timeInterval) const
{
}

} /* namespace msl */
