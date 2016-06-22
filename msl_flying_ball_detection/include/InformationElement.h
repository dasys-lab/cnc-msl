/*
 * InformationElement.h
 *
 *  Created on: May 13, 2014
 *      Author: Stefan Niemczyk
 */

#ifndef INFORMATIONELEMENT_H_
#define INFORMATIONELEMENT_H_

namespace msl
{
	typedef signed long long InfoTime;

/**
 * Information element stores one information and provides required meta
 * data. The information can not be changed.
 *
 */
template<typename T>
  class InformationElement
  {
  public:
    /*!
     * \brief Default constructor
     *
     * Default constructor
     *
     * \param information The information to store.
     * \param timeStamp time when information is received.
     */
    InformationElement(boost::shared_ptr<T> information, InfoTime timeStamp) : information(information), timeStamp(timeStamp)
    {
    }

    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~InformationElement()
    {
//
    }

    /*!
     * \brief Returns the information stored in this container.
     *
     * Returns the information stored in this container.
     */
    boost::shared_ptr<T> getInformation() const
    {
      return this->information;
    }
    unsigned long timeStamp; /**< reception time of the information */
    double certainty;

  private:
    boost::shared_ptr<T> information; /**< the stored information */
  };

} /* namespace ice */

#endif /* INFORMATIONELEMENT_H_ */
