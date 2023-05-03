////////////////////////////////////////////////////////////////////////////
//  MIT License
//
//  Copyright (c) 2023 Hiroki, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "spidev_lib.hpp"


SPI::SPI(const char * p_spidev)
{
  m_spidev = NULL;

  if(p_spidev != NULL )
  {
      m_spidev = (char *)malloc(strlen(p_spidev)+1);
      if(m_spidev != NULL)
        strcpy(m_spidev,p_spidev);
  }
   m_open = false;
}

SPI::~SPI()
{
  if (m_spidev != NULL )
  {
	free(m_spidev);
	m_spidev = NULL;
  }
  if (m_open)
      close(m_spifd);
}

bool SPI::begin(){

    unsigned char SPIMode = SPI_MODE_3;
    unsigned char SPIBits = 8;
    uint32_t SPISpeed = 1000000;
    
    /* open spidev device */
    if (m_open == true )
       return true;
    if (m_spidev == NULL)
       return false;
    m_spifd = open(m_spidev, O_RDWR);
  
    if (m_spifd < 0) 
        return false;

    /* Set SPI_POL and SPI_PHA */
    if (ioctl(m_spifd, SPI_IOC_WR_MODE, &SPIMode) < 0) 
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MODE, &SPIMode) < 0) 
    {
        close(m_spifd);
        return false;
    }

    /* Set bits per word*/
    if (ioctl(m_spifd, SPI_IOC_WR_BITS_PER_WORD, &SPIBits) < 0) 
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_BITS_PER_WORD, &SPIBits) < 0) 
    {
        close(m_spifd);
        return false;
    }

    /* Set SPI speed*/
    if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &SPISpeed) < 0) 
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MAX_SPEED_HZ, &SPISpeed) < 0) 
    {
        close(m_spifd);
        return false;
    }
    m_open = true;

    return true;
}

bool SPI::write(uint8_t regAddr, uint8_t data, const char *errorMsg)
{
    uint8_t tx_buffer[2] = {};
    struct spi_ioc_transfer spi_message[1];

    tx_buffer[0] = regAddr;
    tx_buffer[1] = data;

    memset(spi_message, 0, sizeof(spi_message));
    spi_message[0].tx_buf = (unsigned long)tx_buffer;
    spi_message[0].len = 2;

    if(ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message) < 0)
    {
        if (strlen(errorMsg) > 0)
            ERROR_LOG2(" data write of %d failed - %s\n", regAddr, errorMsg);
        return false;
    }

    return true;
}

bool SPI::read(uint8_t regAddr, uint8_t length, uint8_t *data, const char *errorMsg)
{
    uint8_t rx_buffer[length + 1] = {};
    uint8_t tx_buffer[length + 1] = {};

    struct spi_ioc_transfer spi_message[1];

    tx_buffer[0] = regAddr | 0x80;
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].rx_buf = (unsigned long)rx_buffer;
    spi_message[0].tx_buf = (unsigned long)tx_buffer;
    spi_message[0].len = length + 1;

    if (ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message) < 0)
    {
        if (strlen(errorMsg) > 0)
            ERROR_LOG2("read error from %d - %s\n", regAddr, errorMsg);
        return false;
    }

    for(int i = 0; i < length; i++)
    {
        data[i] = rx_buffer[i+1];
    }

    return true;
}

void SPI::delayMs(int milliSeconds)
{
    usleep(1000 * milliSeconds);
}


