#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <string>

namespace ORB_SLAM3
{

struct Station
{
    //friend class boost::serialization::access;
    std::string id;
    Eigen::Vector3f position;

    Station() = default;
    Station(const std::string &_id, const Eigen::Vector3f &_position)
        : id(_id), position(_position) {}

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & id;
        ar & boost::serialization::make_array(position.data(), position.size());
    }
};

struct Leg
{
    std::string from_station_id;
    std::string to_station_id;
    float distance;
    float azimuth;
    float inclination;
    float distance_noise;
    float azimuth_noise;
    float inclination_noise;
    struct Station *from_station;
    struct Station *to_station;

    Leg() = default;
    Leg(const std::string &_from_station_id, const std::string &_to_station_id,
        float _distance, float _azimuth, float _inclination,
        float _distance_noise, float _azimuth_noise, float _inclination_noise)
        : from_station_id(_from_station_id), to_station_id(_to_station_id),
          distance(_distance), azimuth(_azimuth), inclination(_inclination),
          distance_noise(_distance_noise), azimuth_noise(_azimuth_noise), inclination_noise(_inclination_noise),
          from_station(nullptr), to_station(nullptr) {}
    
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & from_station_id;
        ar & to_station_id;
        ar & distance;
        ar & azimuth;
        ar & inclination;
        ar & distance_noise;
        ar & azimuth_noise;
        ar & inclination_noise;
    }
};


} // namespace ORB_SLAM3
