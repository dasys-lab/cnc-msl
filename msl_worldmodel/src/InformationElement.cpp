#include "InformationElement.h"

namespace msl
{

/**
 * Default constructor
 * @param information The information to store.
 * @param timeStamp Time when the information was created.
 */
template <typename T>
InformationElement<T>::InformationElement(std::shared_ptr<T> information, InfoTime creationTime, InfoTime validityTime, double certainty)
    : information(information)
    , creationTime(creationTime)
    , validityTime(validityTime)
    , certainty(certainty)
    , isDummy(!information)
{
}
template <typename T>
InformationElement<T>::InformationElement()
    : InformationElement(nullptr, 0, 0, 0)
{
}

/**
 * Default destructor
 */
template <typename T>
InformationElement<T>::~InformationElement()
{
}

template <typename T>
InfoTime InformationElement<T>::getCreationTime() const
{
    return this->creationTime;
}

template <typename T>
InfoTime InformationElement<T>::getValidityTime() const
{
    return this->validityTime;
}

template <typename T>
double InformationElement<T>::getCertainty() const
{
    return this->certainty;
}

template <typename T>
bool InformationElement<T>::isValid() const
{
    return !this->isDummy && this->validityTime > 0; // TODO: replace 0 with currentTime;
}

/**
 * Returns the information stored in this container.
 */
template <typename T>
std::shared_ptr<T> InformationElement<T>::getInformation() const
{
    return this->information;
}

} /* namespace msl */
