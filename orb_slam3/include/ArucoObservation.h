// ArucoObservation.h
#pragma once
#include <boost/serialization/access.hpp>

namespace ORB_SLAM3
{

struct ArucoObservation
{
    char id;
    float xLeft, yLeft;
    float xRight;

    ArucoObservation() = default;
    ArucoObservation(char _id, float _xLeft, float _yLeft, float _xRight)
        : id(_id), xLeft(_xLeft), yLeft(_yLeft), xRight(_xRight) {}

    friend class boost::serialization::access;
    
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & id;
        ar & xLeft & yLeft & xRight;
    }
};

} // namespace ORB_SLAM3