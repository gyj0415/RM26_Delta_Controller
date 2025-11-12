//
// Created by fish on 2025/1/3.
//

#include "ctrl_forward_feed.h"

#include <algorithm>

using namespace Controller;

float ForwardFeed::update(float current, float target) {
    return std::clamp(current + target, _output_min, _output_max);
}
