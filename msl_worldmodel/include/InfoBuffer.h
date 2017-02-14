#pragma once

#include <memory>
#include <mutex>
#include <typeinfo>

namespace msl
{
/**
 * Information buffer of shared_ptr of a given data type.
 *
 */
template <typename T> class InfoBuffer
{
  public:
	/**
	 * Default constructor.
	 * @param bufferSize Number of elements which can be stored within the information buffer
	 */
    inline InfoBuffer(const int bufferSize)
    {
        this->bufferSize = bufferSize;
        this->infoElementCounter = 0;
        this->index = -1;
        this->ringBuffer = std::unique_ptr<std::shared_ptr<T>[]>(new std::shared_ptr<T>[this->bufferSize]);
    }

    /**
     *  Default destructor.
     */
    inline virtual ~InfoBuffer()
    {
    }

    /**
     * Returns the last element or NULL if no element exists.
     * @return
     */
    std::shared_ptr<T> getLast()
    {
        return this->getLast(0);
    }

    /**
     * Returns the n-th last element. Returns NULL if no element exists or n > maxBuffer.
     * @param n The n-th last element.
     * @return The n-th last element, 0 will return the newest one.
     */
    std::shared_ptr<T> getLast(int n)
    {
        std::shared_ptr<T> ptr;
        std::lock_guard<std::mutex> guard(mtx_);

        if (this->index < 0 || this->bufferSize <= n || this->infoElementCounter <= n)
        {
            return ptr;
        }

        int index = (this->index - n) % this->bufferSize;

        ptr = this->ringBuffer[index];

        return ptr;
    }

    /**
     * Adds a new element to the information buffer and returns the identifier.
     * @param element The element to add.
     * @return the number of elements in the buffer.
     */
    int add(std::shared_ptr<T> element)
    {
        std::lock_guard<std::mutex> guard(mtx_);

        int index = (++this->index) % this->bufferSize;

        this->ringBuffer[index] = element;

        return this->infoElementCounter++;
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
     * Returns the current count of elements within the information buffer.
     * @return the current count of elements within the information buffer.
     */
    const int getSize() const
    {
        return (this->bufferSize < this->infoElementCounter) ? this->bufferSize : this->infoElementCounter;
    }

    /*!
     * \brief Clears the buffer.
     *
     * Clears the buffer. If cleanBuffer is false only the index structure is reseted, but the
     * buffer still exists (old elements are not accessible). If cleanBuffer is true the
     * pointers from the buffer are cleared as well.
     *
     * \param cleanBuffer True to clear the buffer.
     */
    int clear(bool cleanBuffer)
    {
        std::lock_guard<std::mutex> guard(mtx_);
        this->index = -1;
        this->infoElementCounter = 0;

        if (false == cleanBuffer)
            return 0;

        std::shared_ptr<T> ptr;
        for (int i = 0; i < this->bufferSize; ++i)
            this->ringBuffer[i] = ptr;

        return 0;
    }

    /*!
     *\brief Returns the type_info of the used template type.
     *
     * Returns the type_info of the used template type.
     */
    const std::type_info *getTypeInfo() const
    {
        return &typeid(T);
    }

  private:
    std::mutex mtx_;
    std::unique_ptr<std::shared_ptr<T>[]> ringBuffer; /**< Ring buffer of elements */
    int bufferSize;                                   /**< number of stored elements */
    ulong infoElementCounter;                         /**< Counter of elements added to the ring buffer */
    long index;                                       /**< Current index of the last added element */
};

} /* namespace ice */
