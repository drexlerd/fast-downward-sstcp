#ifndef UTILS_LOGGING_H
#define UTILS_LOGGING_H

#include "system.h"
#include "timer.h"

#include <ostream>
#include <string>
#include <vector>
#include <unordered_set>

namespace utils {
/*
  Simple logger that prepends time and peak memory info to messages.
  Logs are written to stdout.

  Usage:
        utils::g_log << "States: " << num_states << endl;
*/
struct Log {
    template<typename T>
    std::ostream &operator<<(const T &elem) {
        return std::cout << "[t=" << g_timer << ", "
                         << get_peak_memory_in_kb() << " KB] " << elem;
    }
};

extern Log g_log;

class TraceBlock {
    std::string block_name;
public:
    explicit TraceBlock(const std::string &block_name);
    ~TraceBlock();
};

extern void trace(const std::string &msg = "");
}

namespace std {
template<class T>
ostream &operator<<(ostream &stream, const vector<T> &vec) {
    stream << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i != 0)
            stream << ", ";
        stream << vec[i];
    }
    stream << "]";
    return stream;
}

template<class T>
ostream &operator<<(ostream &stream, const unordered_set<T> &set) {
    stream << "{";
    for (const T &element : set) {            
        stream << element;
        stream << ", ";
    }
    stream << "}";
    return stream;
}

}

#endif
