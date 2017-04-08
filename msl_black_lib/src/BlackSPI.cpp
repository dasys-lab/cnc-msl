 /*

 ####################################################################################
 #  BlackLib Library controls Beaglebone Black's inputs and outputs.                #
 #  Copyright (C) 2013-2014 by Yigit YUCE                                           #
 ####################################################################################
 #                                                                                  #
 #  This file is part of BlackLib library.                                          #
 #                                                                                  #
 #  BlackLib library is free software: you can redistribute it and/or modify        #
 #  it under the terms of the GNU Lesser General Public License as published by     #
 #  the Free Software Foundation, either version 3 of the License, or               #
 #  (at your option) any later version.                                             #
 #                                                                                  #
 #  BlackLib library is distributed in the hope that it will be useful,             #
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of                  #
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                   #
 #  GNU Lesser General Public License for more details.                             #
 #                                                                                  #
 #  You should have received a copy of the GNU Lesser General Public License        #
 #  along with this program.  If not, see <http://www.gnu.org/licenses/>.           #
 #                                                                                  #
 #  For any comment or suggestion please contact the creator of BlackLib Library    #
 #  at ygtyce@gmail.com                                                             #
 #                                                                                  #
 ####################################################################################

 */

#include <iostream>


#include "BlackSPI.h"

namespace BlackLib
{


    BlackSPI::BlackSPI(spiName spi)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= true;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );


        this->loadDeviceTree();
        this->findPortPath();

    }

    BlackSPI::BlackSPI(spiName spi, BlackSpiProperties spiProperties)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= false;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );

        constructorProperties   = spiProperties;

        this->loadDeviceTree();
        this->findPortPath();

    }

    BlackSPI::BlackSPI(spiName spi, uint8_t spiBitsPerWord, uint8_t spiMode, uint32_t spiSpeed)
    {
        this->spiChipNumber     = (static_cast<int>(spi) % 2);
        this->spiBusNumber      = ( (static_cast<int>(spi) - this->spiChipNumber)/2 );

        this->dtSpiFilename     = "BLACKLIB-SPI" + tostr(this->spiBusNumber);
        this->spiFD             = -1;
        this->isOpenFlag        = false;
        this->isCurrentEqDefault= false;
        this->spiErrors         = new errorSPI( this->getErrorsFromCore() );

std::cout << "spiBpW: " << (int) spiBitsPerWord << std::endl;
std::cout << "spiMode: " << (int) spiMode << std::endl;
std::cout << "spiSpeed: " << (int) spiSpeed << std::endl;

        constructorProperties.spiBitsPerWord    = spiBitsPerWord;
        constructorProperties.spiMode           = spiMode;
        constructorProperties.spiSpeed          = spiSpeed;

        this->loadDeviceTree();
        this->findPortPath();
    }

    BlackSPI::~BlackSPI()
    {
        this->close();
        delete this->spiErrors;
    }






    bool        BlackSPI::loadDeviceTree()
    {
        std::string file = this->getSlotsFilePath();

        std::ofstream slotsFile;
        slotsFile.open(file.c_str(),std::ios::out);
        if(slotsFile.fail())
        {
            this->spiErrors->dtError = true;
            slotsFile.close();
            return false;
        }
        else
        {
            this->spiErrors->dtError = false;
            slotsFile << this->dtSpiFilename;
            slotsFile.close();
            return true;
        }
    }

    bool        BlackSPI::findPortPath()
    {
        std::string limitedSearchResult;

        if( this->spiBusNumber == 0 )
        {
            limitedSearchResult = this->searchDirectoryOcp(BlackCore::SPI0);
std::cout << "lim: " << limitedSearchResult << std::endl;
        }
        else if( this->spiBusNumber == 1 )
        {
            limitedSearchResult = this->searchDirectoryOcp(BlackCore::SPI1);
        }
        else
        {
            this->spiErrors->portPathError = true;
            return false;
        }


        if( limitedSearchResult == SEARCH_DIR_NOT_FOUND )
        {
            this->spiErrors->portPathError = true;
            return false;
        }
        else if( ::isdigit( static_cast<int>(limitedSearchResult[3]) ) == 0 )
        {
            this->spiErrors->portPathError = true;
            return false;
        }
        else
        {
            this->spiErrors->portPathError = false;
            this->spiPortPath = "/dev/spidev" + tostr( limitedSearchResult[3] ) + "." + tostr(this->spiChipNumber);
std::cout << "portPath: " << this->spiPortPath << std::endl;
            return true;
        }

    }


    bool        BlackSPI::open(uint openMode)
    {
        uint flags = 0;

std::cout << "Mode: " << openMode << std::endl;

        if( (openMode & ReadOnly)   == ReadOnly     ){  flags |= O_RDONLY;  }
        if( (openMode & WriteOnly)  == WriteOnly    ){  flags |= O_WRONLY;  }
        if( (openMode & ReadWrite)  == ReadWrite    ){  flags |= O_RDWR;    }
        if( (openMode & Append)     == Append       ){  flags |= O_APPEND;  }
        if( (openMode & Truncate)   == Truncate     ){  flags |= O_TRUNC;   }
        if( (openMode & NonBlock)   == NonBlock     ){  flags |= O_NONBLOCK;}

std::cout << "PortPath: " << spiPortPath.c_str() << std::endl;
std::cout << "Flag: " << flags << std::endl;

        this->spiFD = ::open(this->spiPortPath.c_str(), flags);

std::cout << "spiFD: " << this->spiFD << std::endl;

        if( this->spiFD < 0 )
        {
            this->spiErrors->openError  = true;
            this->isOpenFlag            = false;
            return false;
        }


        this->spiErrors->openError  = false;
        this->isOpenFlag            = true;
        this->defaultProperties     = this->getProperties();

        if( this->isCurrentEqDefault )
        {
            this->currentProperties = this->defaultProperties;
        }
        else
        {
            if( this->setProperties( this->constructorProperties ))
            {
                this->currentProperties = this->constructorProperties;
            }
            else
            {
                this->currentProperties = this->defaultProperties;
            }
        }
std::cout << "cur BpW: " << (int) this->currentProperties.spiBitsPerWord << std::endl;
std::cout << "cur Mode: " << (int) this->currentProperties.spiMode << std::endl;
std::cout << "cur Speed: " << this->currentProperties.spiSpeed << std::endl;
        return true;
    }

    bool        BlackSPI::close()
    {
        if( ::close(this->spiFD) < 0 )
        {
            this->spiErrors->closeError = true;
            return false;
        }
        else
        {
            this->spiErrors->closeError = false;
            this->isOpenFlag = false;
            return true;
        }
    }








    bool        BlackSPI::setMode(uint8_t newMode)
    {
std::cout << "setMode: " << (int) newMode << std::endl;
       if( ::ioctl(this->spiFD, SPI_IOC_WR_MODE, &newMode) == -1 )
       {
           this->spiErrors->modeError = true;
std::cout << "setMode: " << "false" << std::endl;
           return false;
       }
       else
       {
           this->spiErrors->modeError = false;
           this->currentProperties.spiMode = newMode;
std::cout << "setMode: " << "true" << std::endl;
           return true;
       }
    }

    uint8_t     BlackSPI::getMode()
    {
        uint8_t mode;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_MODE, &mode) == -1 )
        {
            this->spiErrors->modeError = true;
            return 0;
        }
        else
        {
            this->spiErrors->modeError = false;
            this->currentProperties.spiMode = mode;
            return mode;
        }
    }



    bool        BlackSPI::setMaximumSpeed(uint32_t newSpeed)
    {
std::cout << "setSpeed: " << (int) newSpeed << std::endl;
       if( ::ioctl(this->spiFD, SPI_IOC_WR_MAX_SPEED_HZ, &newSpeed) == -1 )
       {
           this->spiErrors->speedError = true;
           return false;
       }
       else
       {
std::cout << "setSpeed: true" << std::endl;
           this->spiErrors->speedError = false;
           this->currentProperties.spiSpeed = newSpeed;
           return true;
       }
    }

    uint32_t    BlackSPI::getMaximumSpeed()
    {
        uint32_t speed;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1 )
        {
            this->spiErrors->speedError = true;
            return 0;
        }
        else
        {
            this->spiErrors->speedError = false;
            this->currentProperties.spiSpeed = speed;
            return speed;
        }
    }



    bool        BlackSPI::setBitsPerWord(uint8_t newBitSize)
    {
std::cout << "setBpW: " << (int) newBitSize << std::endl;
       if( ::ioctl(this->spiFD, SPI_IOC_WR_BITS_PER_WORD, &newBitSize) == -1 )
       {
           this->spiErrors->bitSizeError = true;
           return false;
       }
       else
       {
std::cout << "setBpW: true" << std::endl;
           this->spiErrors->bitSizeError = false;
           this->currentProperties.spiBitsPerWord = newBitSize;
           return true;
       }
    }

    uint8_t     BlackSPI::getBitsPerWord()
    {
        uint8_t bitsSize;

        if( ::ioctl(this->spiFD, SPI_IOC_RD_BITS_PER_WORD, &bitsSize) == -1 )
        {
            this->spiErrors->bitSizeError = true;
            return 0;
        }
        else
        {
            this->spiErrors->bitSizeError = false;
            this->currentProperties.spiBitsPerWord = bitsSize;
            return bitsSize;
        }
    }



    bool        BlackSPI::setProperties(BlackSpiProperties &newProperties)
    {
        return ( this->setBitsPerWord( newProperties.spiBitsPerWord ) and
                 this->setMaximumSpeed( newProperties.spiSpeed ) and
                 this->setMode( newProperties.spiMode )
               );
    }

    BlackSpiProperties BlackSPI::getProperties()
    {
        this->getBitsPerWord();
        this->getMode();
        this->getMaximumSpeed();
        return ( this->currentProperties );
    }



    uint8_t     BlackSPI::transfer(uint8_t writeByte, uint16_t wait_us)
    {
        uint8_t tempReadByte = 0x00;


        if( ! this->isOpenFlag )
        {
            this->spiErrors->openError      = true;
            this->spiErrors->transferError  = true;
            return tempReadByte;
        }



        this->spiErrors->openError          = false;
        spi_ioc_transfer package;

        package.tx_buf          = (unsigned long)&writeByte;
        package.rx_buf          = (unsigned long)&tempReadByte;
        package.len             = 1;
        package.delay_usecs     = wait_us;
        package.speed_hz        = this->currentProperties.spiSpeed;
        package.bits_per_word   = this->currentProperties.spiBitsPerWord;

std::cout << "TX_BUF: " << package.tx_buf << ", RX_BUF: " << package.rx_buf << std::endl;
std::cout << "Len: " << package.len << ", Speed_hz: " << package.speed_hz << std::endl;
std::cout << "Delay: " << package.delay_usecs << ", BpW: " << package.bits_per_word << std::endl;
std::cout << "CS_C: " << package.cs_change << ", Pad: " << package.pad << std::endl;
std::cout << "TX_N: " << package.tx_nbits << ", RX_N: " << package.rx_nbits << std::endl;

int var_ioctl = 0;

        /*if(*/var_ioctl = ::ioctl(this->spiFD, SPI_IOC_MESSAGE(1), &package);// >= 0)
	if(var_ioctl >= 0)
        {
            this->spiErrors->transferError = false;
std::cout << "Tx okay" << std::endl;
            return tempReadByte;
        }
        else
        {
            this->spiErrors->transferError = true;
std::cout << "Tx failed: " << var_ioctl << std::endl;
            return tempReadByte;
        }
    }

    bool        BlackSPI::transfer(uint8_t *writeBuffer, uint8_t *readBuffer, size_t bufferSize, uint16_t wait_us)
    {
        if( ! this->isOpenFlag )
        {
            this->spiErrors->openError      = true;
            this->spiErrors->transferError  = true;
            return false;
        }



        this->spiErrors->openError          = false;
        uint8_t tempReadBuffer[ bufferSize ];
        memset( tempReadBuffer, 0, bufferSize);

        spi_ioc_transfer package;
        package.tx_buf          = (unsigned long)writeBuffer;
        package.rx_buf          = (unsigned long)tempReadBuffer;
        package.len             = bufferSize;
        package.delay_usecs     = wait_us;
        package.speed_hz        = this->currentProperties.spiSpeed;
        package.bits_per_word   = this->currentProperties.spiBitsPerWord;

std::cout << "TX_BUF: " << package.tx_buf << ", RX_BUF: " << package.rx_buf << std::endl;
std::cout << "Len: " << package.len << ", Speed_hz: " << package.speed_hz << std::endl;
std::cout << "Delay: " << package.delay_usecs << ", BpW: " << (int) package.bits_per_word << std::endl;
std::cout << "CS_C: " << (int) package.cs_change << ", Pad: " << package.pad << std::endl;
std::cout << "TX_N: " << (int) package.tx_nbits << ", RX_N: " << (int) package.rx_nbits << std::endl;

        if( ::ioctl(this->spiFD, SPI_IOC_MESSAGE(1), &package) >= 0)
        {
            this->spiErrors->transferError = false;
            memcpy(readBuffer, tempReadBuffer, bufferSize);
std::cout << "Transfer Buffer: " << "=)" << std::endl;
            return true;
        }
        else
        {
            this->spiErrors->transferError = true;
std::cout << "Transfer Buffer: " << "ERROR =(" << std::endl;
            return false;
        }
    }






    std::string BlackSPI::getPortName()
    {
        return this->spiPortPath;
    }

    bool        BlackSPI::isOpen()
    {
        return this->isOpenFlag;
    }

    bool        BlackSPI::isClose()
    {
        return !(this->isOpenFlag);
    }


    bool        BlackSPI::fail()
    {
        return (this->spiErrors->dtError or
                this->spiErrors->openError or
                this->spiErrors->closeError or
                this->spiErrors->portPathError or
                this->spiErrors->transferError or
                this->spiErrors->modeError or
                this->spiErrors->speedError or
                this->spiErrors->bitSizeError
                );
    }

    bool        BlackSPI::fail(BlackSPI::flags f)
    {
        if(f==dtErr)            { return this->spiErrors->dtError;         }
        if(f==openErr)          { return this->spiErrors->openError;       }
        if(f==closeErr)         { return this->spiErrors->closeError;      }
        if(f==portPathErr)      { return this->spiErrors->portPathError;   }
        if(f==transferErr)      { return this->spiErrors->transferError;   }
        if(f==modeErr)          { return this->spiErrors->modeError;       }
        if(f==speedErr)         { return this->spiErrors->speedError;      }
        if(f==bitSizeErr)       { return this->spiErrors->bitSizeError;    }

        return true;
    }


} /* namespace BlackLib */
