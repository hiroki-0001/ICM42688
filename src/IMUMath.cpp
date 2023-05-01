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

#include "IMUMath.hpp"


uint64_t IMUMath::currentUSecsSinceEpoch()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
}


//----------------------------------------------------------
//  The Vector3 class
//----------------------------------------------------------

Vector3::Vector3()
{
    zero();
}

Vector3::Vector3(float x, float y, float z)
{
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

const Vector3& Vector3::operator +=(Vector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] += vec.m_data[i];
    return *this;
}

const Vector3& Vector3::operator -=(Vector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] -= vec.m_data[i];
    return *this;
}

Vector3& Vector3::operator =(const Vector3& vec)
{
    if (this == &vec)
        return *this;

    m_data[0] = vec.m_data[0];
    m_data[1] = vec.m_data[1];
    m_data[2] = vec.m_data[2];

    return *this;
}

void Vector3::zero()
{
    for (int i = 0; i < 3; i++)
        m_data[i] = 0;
}

void Vector3::normalize()
{
    float length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);

    if (length == 0)
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
}

float Vector3::length()
{
    return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);
}

void Vector3::convertToVector(unsigned char *rawData, Vector3& vec, float scale, float *Bias)
{
    vec.setX(((float)((int16_t)(((uint16_t)rawData[0] << 8) | (uint16_t)rawData[1])) * scale) - Bias[0]);
    vec.setY(((float)((int16_t)(((uint16_t)rawData[2] << 8) | (uint16_t)rawData[3])) * scale) - Bias[1]);
    vec.setZ(((float)((int16_t)(((uint16_t)rawData[4] << 8) | (uint16_t)rawData[5])) * scale) - Bias[2]);
}

float Vector3::dotProduct(const Vector3& a, const Vector3& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

void Vector3::crossProduct(const Vector3& a, const Vector3& b, Vector3& d)
{
    d.setX(a.y() * b.z() - a.z() * b.y());
    d.setY(a.z() * b.x() - a.x() * b.z());
    d.setZ(a.x() * b.y() - a.y() * b.x());
}


void Vector3::accelToEuler(Vector3& rollPitchYaw) const
{
    Vector3 normAccel = *this;

    normAccel.normalize();

    rollPitchYaw.setX(atan2(normAccel.y(), normAccel.z()));
    rollPitchYaw.setY(-atan2(normAccel.x(), sqrt(normAccel.y() * normAccel.y() + normAccel.z() * normAccel.z())));
    rollPitchYaw.setZ(0);
}


void Vector3::accelToQuaternion(Quaternion& qPose) const
{
    Vector3 normAccel = *this;
    Vector3 vec;
    Vector3 z(0, 0, 1.0);

    normAccel.normalize();

    float angle = acos(Vector3::dotProduct(z, normAccel));
    Vector3::crossProduct(normAccel, z, vec);
    vec.normalize();

    qPose.fromAngleVector(angle, vec);
}

//----------------------------------------------------------
//  The Quaternion class
//----------------------------------------------------------

Quaternion::Quaternion()
{
    zero();
}

Quaternion::Quaternion(float scalar, float x, float y, float z)
{
    m_data[0] = scalar;
    m_data[1] = x;
    m_data[2] = y;
    m_data[3] = z;
}

Quaternion& Quaternion::operator =(const Quaternion& quat)
{
    if (this == &quat)
        return *this;

    m_data[0] = quat.m_data[0];
    m_data[1] = quat.m_data[1];
    m_data[2] = quat.m_data[2];
    m_data[3] = quat.m_data[3];

    return *this;
}

Quaternion& Quaternion::operator +=(const Quaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] += quat.m_data[i];
    return *this;
}

Quaternion& Quaternion::operator -=(const Quaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= quat.m_data[i];
    return *this;
}

Quaternion& Quaternion::operator -=(const float val)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= val;
    return *this;
}

Quaternion& Quaternion::operator *=(const Quaternion& qb)
{
    Quaternion qa;

    qa = *this;

    m_data[0] = qa.scalar() * qb.scalar() - qa.x() * qb.x() - qa.y() * qb.y() - qa.z() * qb.z();
    m_data[1] = qa.scalar() * qb.x() + qa.x() * qb.scalar() + qa.y() * qb.z() - qa.z() * qb.y();
    m_data[2] = qa.scalar() * qb.y() - qa.x() * qb.z() + qa.y() * qb.scalar() + qa.z() * qb.x();
    m_data[3] = qa.scalar() * qb.z() + qa.x() * qb.y() - qa.y() * qb.x() + qa.z() * qb.scalar();

    return *this;
}


Quaternion& Quaternion::operator *=(const float val)
{
    m_data[0] *= val;
    m_data[1] *= val;
    m_data[2] *= val;
    m_data[3] *= val;

    return *this;
}


const Quaternion Quaternion::operator *(const Quaternion& qb) const
{
    Quaternion result = *this;
    result *= qb;
    return result;
}

const Quaternion Quaternion::operator *(const float val) const
{
    Quaternion result = *this;
    result *= val;
    return result;
}


const Quaternion Quaternion::operator -(const Quaternion& qb) const
{
    Quaternion result = *this;
    result -= qb;
    return result;
}

const Quaternion Quaternion::operator -(const float val) const
{
    Quaternion result = *this;
    result -= val;
    return result;
}

void Quaternion::zero()
{
    for (int i = 0; i < 4; i++)
        m_data[i] = 0;
}

void Quaternion::normalize()
{
    float length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3]);

    if ((length == 0) || (length == 1))
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
    m_data[3] /= length;
}

void Quaternion::toEuler(Vector3& vec)
{
    vec.setX(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
            1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

    vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

    vec.setZ(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
            1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));
}

void Quaternion::fromEuler(Vector3& vec)
{
    float cosX2 = cos(vec.x() / 2.0f);
    float sinX2 = sin(vec.x() / 2.0f);
    float cosY2 = cos(vec.y() / 2.0f);
    float sinY2 = sin(vec.y() / 2.0f);
    float cosZ2 = cos(vec.z() / 2.0f);
    float sinZ2 = sin(vec.z() / 2.0f);

    m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    normalize();
}

void Quaternion::toAngleVector(float& angle, Vector3& vec)
{
    float halfTheta;
    float sinHalfTheta;

    halfTheta = acos(m_data[0]);
    sinHalfTheta = sin(halfTheta);

    if (sinHalfTheta == 0) {
        vec.setX(1.0);
        vec.setY(0);
        vec.setZ(0);
    } else {
        vec.setX(m_data[1] / sinHalfTheta);
        vec.setY(m_data[1] / sinHalfTheta);
        vec.setZ(m_data[1] / sinHalfTheta);
    }
    angle = 2.0 * halfTheta;
}

void Quaternion::fromAngleVector(const float& angle, const Vector3& vec)
{
    float sinHalfTheta = sin(angle / 2.0);
    m_data[0] = cos(angle / 2.0);
    m_data[1] = vec.x() * sinHalfTheta;
    m_data[2] = vec.y() * sinHalfTheta;
    m_data[3] = vec.z() * sinHalfTheta;
}

Quaternion Quaternion::conjugate() const
{
    Quaternion q;
    q.setScalar(m_data[0]);
    q.setX(-m_data[1]);
    q.setY(-m_data[2]);
    q.setZ(-m_data[3]);
    return q;
}


//----------------------------------------------------------
//  The Matrix4x4 class
//----------------------------------------------------------

Matrix4x4::Matrix4x4()
{
    fill(0);
}

Matrix4x4& Matrix4x4::operator =(const Matrix4x4& mat)
{
    if (this == &mat)
        return *this;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = mat.m_data[row][col];

    return *this;
}


void Matrix4x4::fill(float val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = val;
}


Matrix4x4& Matrix4x4::operator +=(const Matrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] += mat.m_data[row][col];

    return *this;
}

Matrix4x4& Matrix4x4::operator -=(const Matrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] -= mat.m_data[row][col];

    return *this;
}

Matrix4x4& Matrix4x4::operator *=(const float val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] *= val;

    return *this;
}

const Matrix4x4 Matrix4x4::operator +(const Matrix4x4& mat) const
{
    Matrix4x4 result = *this;
    result += mat;
    return result;
}

const Matrix4x4 Matrix4x4::operator *(const float val) const
{
    Matrix4x4 result = *this;
    result *= val;
    return result;
}


const Matrix4x4 Matrix4x4::operator *(const Matrix4x4& mat) const
{
    Matrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[row][col] =
                    m_data[row][0] * mat.m_data[0][col] +
                    m_data[row][1] * mat.m_data[1][col] +
                    m_data[row][2] * mat.m_data[2][col] +
                    m_data[row][3] * mat.m_data[3][col];

    return res;
}


const Quaternion Matrix4x4::operator *(const Quaternion& q) const
{
    Quaternion res;

    res.setScalar(m_data[0][0] * q.scalar() + m_data[0][1] * q.x() + m_data[0][2] * q.y() + m_data[0][3] * q.z());
    res.setX(m_data[1][0] * q.scalar() + m_data[1][1] * q.x() + m_data[1][2] * q.y() + m_data[1][3] * q.z());
    res.setY(m_data[2][0] * q.scalar() + m_data[2][1] * q.x() + m_data[2][2] * q.y() + m_data[2][3] * q.z());
    res.setZ(m_data[3][0] * q.scalar() + m_data[3][1] * q.x() + m_data[3][2] * q.y() + m_data[3][3] * q.z());

    return res;
}

void Matrix4x4::setToIdentity()
{
    fill(0);
    m_data[0][0] = 1;
    m_data[1][1] = 1;
    m_data[2][2] = 1;
    m_data[3][3] = 1;
}

Matrix4x4 Matrix4x4::transposed()
{
    Matrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[col][row] = m_data[row][col];
    return res;
}

//  Note:
//  The matrix inversion code here was strongly influenced by some old code I found
//  but I have no idea where it came from. Apologies to whoever wrote it originally!
//  If it's you, please let me know at info@richards-tech.com so I can credit it correctly.

Matrix4x4 Matrix4x4::inverted()
{
    Matrix4x4 res;

    float det = matDet();

    if (det == 0) {
        res.setToIdentity();
        return res;
    }

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if ((row + col) & 1)
                res.m_data[col][row] = -matMinor(row, col) / det;
            else
                res.m_data[col][row] = matMinor(row, col) / det;
        }
    }

    return res;
}

float Matrix4x4::matDet()
{
    float det = 0;

    det += m_data[0][0] * matMinor(0, 0);
    det -= m_data[0][1] * matMinor(0, 1);
    det += m_data[0][2] * matMinor(0, 2);
    det -= m_data[0][3] * matMinor(0, 3);
    return det;
}

float Matrix4x4::matMinor(const int row, const int col)
{
    static int map[] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};

    int *rc;
    int *cc;
    float res = 0;

    rc = map + row * 3;
    cc = map + col * 3;

    res += m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[2]];
    res -= m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[2]];
    res += m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[0]];
    res += m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[0]];
    return res;
}