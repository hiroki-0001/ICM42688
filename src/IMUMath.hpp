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

#ifndef _IMUMATH_H_
#define _IMUMATH_H_

#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <sys/time.h>

#define	PI					3.1415926535
#define	DEGREE_TO_RAD		(PI / 180.0)
#define	RAD_TO_DEGREE		(180.0 / PI)

class Vector3;
class Quaternion;
class Matrix4x4;

class IMUMath
{
    public:
        //  currentUSecsSinceEpoch() is the source of all timestamps and
        //  is the number of uS since the standard epoch
        static uint64_t currentUSecsSinceEpoch();

};

class Vector3
{
    public:
        Vector3();
        Vector3(float x, float y, float z);

        const Vector3&  operator +=(Vector3& vec);
        const Vector3&  operator -=(Vector3& vec);
        Vector3& operator =(const Vector3& vec);

        void zero();
        void normalize();
        float length();
        static void convertToVector(unsigned char *rawData, Vector3& vec, float scale, float *Bias);

        static float dotProduct(const Vector3& a, const Vector3& b);
        static void crossProduct(const Vector3& a, const Vector3& b, Vector3& d);
        void accelToEuler(Vector3& rollPitchYaw) const;
        void accelToQuaternion(Quaternion& qPose) const;

        inline float x() const { return m_data[0]; }
        inline float y() const { return m_data[1]; }
        inline float z() const { return m_data[2]; }
        inline float data(const int i) const { return m_data[i]; }

        inline void setX(const float val) { m_data[0] = val; }
        inline void setY(const float val) { m_data[1] = val; }
        inline void setZ(const float val) { m_data[2] = val; }
        inline void setData(const int i, float val) { m_data[i] = val; }
        inline void fromArray(float *val) { std::memcpy(m_data, val, 3 * sizeof(float)); }
        inline void toArray(float *val) const { std::memcpy(val, m_data, 3 * sizeof(float)); }

    private:
        float m_data[3];
};

class Quaternion
{
    public:
        Quaternion();
        Quaternion(float scalar, float x, float y, float z);

        Quaternion& operator +=(const Quaternion& quat);
        Quaternion& operator -=(const Quaternion& quat);
        Quaternion& operator *=(const Quaternion& qb);
        Quaternion& operator *=(const float val);
        Quaternion& operator -=(const float val);

        Quaternion& operator =(const Quaternion& quat);
        const Quaternion operator *(const Quaternion& qb) const;
        const Quaternion operator *(const float val) const;
        const Quaternion operator -(const Quaternion& qb) const;
        const Quaternion operator -(const float val) const;

        void zero();
        void normalize();
        float length();
        void toEuler(Vector3& vec);
        void fromEuler(Vector3& vec);
        void toAngleVector(float& angle, Vector3& vec);
        void fromAngleVector(const float& angle, const Vector3& vec);

        Quaternion conjugate() const;

        inline float scalar() const { return m_data[0]; }
        inline float x() const { return m_data[1]; }
        inline float y() const { return m_data[2]; }
        inline float z() const { return m_data[3]; }
        inline float data(const int i) const { return m_data[i]; }

        inline void setScalar(const float val) { m_data[0] = val; }
        inline void setX(const float val) { m_data[1] = val; }
        inline void setY(const float val) { m_data[2] = val; }
        inline void setZ(const float val) { m_data[3] = val; }
        inline void setData(const int i, float val) { m_data[i] = val; }
        inline void fromArray(float *val) { memcpy(m_data, val, 4 * sizeof(float)); }
        inline void toArray(float *val) const { memcpy(val, m_data, 4 * sizeof(float)); }

    private:
        float m_data[4];
};

class Matrix4x4
{
    public:
        Matrix4x4();

        Matrix4x4& operator +=(const Matrix4x4& mat);
        Matrix4x4& operator -=(const Matrix4x4& mat);
        Matrix4x4& operator *=(const float val);

        Matrix4x4& operator =(const Matrix4x4& vec);
        const Quaternion operator *(const Quaternion& q) const;
        const Matrix4x4 operator *(const float val) const;
        const Matrix4x4 operator *(const Matrix4x4& mat) const;
        const Matrix4x4 operator +(const Matrix4x4& mat) const;

        inline float val(int row, int col) const { return m_data[row][col]; }
        inline void setVal(int row, int col, float val) { m_data[row][col] = val; }
        void fill(float val);
        void setToIdentity();

        Matrix4x4 inverted();
        Matrix4x4 transposed();
        
    private:
        float m_data[4][4];                                   // row, column
        float matDet();
        float matMinor(const int row, const int col);
};


#endif /* _IMUMATH_H_ */