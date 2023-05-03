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

#ifndef _SPI_LIB_H
#define _SPI_LIB_H

#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>

#define MESSAGE_LOG(m) { printf("%s\n", m); fflush(stdout); }
#define MESSAGE_LOG1(m, x) { printf(m, x); fflush(stdout); }
#define MESSAGE_LOG2(m, x, y) { printf(m, x, y); fflush(stdout); }
#define MESSAGE_LOG3(m, x, y, z) { printf(m, x, y, z); fflush(stdout); }
#define MESSAGE_LOG4(m, x, y, z, a) { printf(m, x, y, z, a); fflush(stdout); }
#define MESSAGE_LOG5(m, x, y, z, a, b) { printf(m, x, y, z, a, b); fflush(stdout); }
#define ERROR_LOG(m)    fprintf(stderr, m);
#define ERROR_LOG1(m, x)    fprintf(stderr, m, x);
#define ERROR_LOG2(m, x, y)    fprintf(stderr, m, x, y);
#define ERROR_LOG3(m, x, y, z)    fprintf(stderr, m, x, y, z);
#define ERROR_LOG4(m, x, y, z, a)    fprintf(stderr, m, x, y, z, a);

// #define MESSAGE_LOG(m)
// #define MESSAGE_LOG1(m, x)
// #define MESSAGE_LOG2(m, x, y)
// #define MESSAGE_LOG3(m, x, y, z)
// #define MESSAGE_LOG4(m, x, y, z, a)
// #define MESSAGE_LOG5(m, x, y, z, a, b)
// #define ERROR_LOG(m)
// #define ERROR_LOG1(m, x)
// #define ERROR_LOG2(m, x, y)
// #define ERROR_LOG3(m, x, y, z)
// #define ERROR_LOG4(m, x, y, z, a)

class SPI
{
    public:
            SPI(const char * p_spidev);
            ~SPI();
            bool begin();
            bool read(uint8_t regAddr, uint8_t length, uint8_t *data, const char *errorMsg);
            bool write(uint8_t regAddr, uint8_t data, const char *errorMsg);
            void delayMs(int milliSeconds);

    private:
            char *m_spidev;
            int m_spifd;
            bool m_open;

};

#endif // _SPI_LIB_H