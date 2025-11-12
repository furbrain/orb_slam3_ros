/**
 * Lightweight logging helpers for ORB_SLAM3.
 *
 * This header was split out from System.h to allow header-only logging
 * in other headers (e.g. Settings.h) without introducing circular includes.
 */

#ifndef ORB_SLAM3_VERBOSE_H
#define ORB_SLAM3_VERBOSE_H

#include <string>
#include <sstream>
#include <iostream>

namespace ORB_SLAM3 {

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;
    using PrintFunc = void(*)(const std::string&, eLevel);
    static PrintFunc customPrint;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if (customPrint)
        {
            customPrint(str, lev);
        } else {
            if(lev <= th)
                std::cout << str << std::endl;
        }
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

// Small helper to build a verbose message using stream syntax and send it
// to Verbose::PrintMess on destruction. Implemented inline so it can be used
// across the codebase without extra link steps.
class VerboseStream {
public:
    explicit VerboseStream(Verbose::eLevel lev) : lev_(lev) {}
    ~VerboseStream() { Verbose::PrintMess(oss_.str(), lev_); }

    template <typename T>
    VerboseStream &operator<<(const T &v) {
        oss_ << v;
        return *this;
    }

    // manipulators like std::endl
    VerboseStream &operator<<(std::ostream &(*func)(std::ostream &)) {
        func(oss_);
        return *this;
    }

    // manipulators like std::setprecision
    VerboseStream &operator<<(std::ios_base &(*func)(std::ios_base &)) {
        func(oss_);
        return *this;
    }

private:
    std::ostringstream oss_;
    Verbose::eLevel lev_;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_VERBOSE_H
