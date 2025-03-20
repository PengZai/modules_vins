#pragma once

#include <iostream>

namespace modules_vins{

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};


struct tuple_for_three_hash {
    template <typename T>
    std::size_t operator()(const T& t) const {
        return std::hash<int>()(std::get<0>(t)) ^
                (std::hash<int>()(std::get<1>(t)) << 1) ^
                (std::hash<int>()(std::get<2>(t)) << 2);
    }
};


} //namespace modules_vins



