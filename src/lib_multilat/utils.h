#pragma once

#include <algorithm>

namespace Ranges
{
    template<typename Range, typename Function>
    Function ForEach(Range& range, Function func)
    {
        return std::for_each(std::begin(range), std::end(range), func);
    }

    template<typename T, typename Function>
    std::vector<T> TransformVector(const std::vector<T>& input, Function func)
    {
        std::vector<T> result;
        result.reserve(input.size());
        std::transform(std::begin(input), std::end(input), std::back_inserter(result), func);
        return result;
    }

    template<typename OutputContainer, typename InputContainer, typename Function>
    OutputContainer Transform(const InputContainer& input, Function func)
    {
        OutputContainer result;
        std::transform(std::begin(input), std::end(input), std::back_inserter(result), func);
        return result;
    }
}