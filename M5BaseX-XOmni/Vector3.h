#ifndef VECTOR_3_H_
#define VECTOR_3_H_

#include <Message.hpp>

typedef struct Vector3Type
{
    float x;
    float y;
    float z;
} vector3_t;

typedef sb::Message<vector3_t> Vector3;

#endif