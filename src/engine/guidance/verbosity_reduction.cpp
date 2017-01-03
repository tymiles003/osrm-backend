#include "engine/guidance/verbosity_reduction.hpp"
#include "engine/guidance/collapsing_utility.hpp"

#include "util/debug.hpp"

#include <iterator>

namespace osrm
{
namespace engine
{
namespace guidance
{
std::vector<RouteStep> suppressShortNameSegments(std::vector<RouteStep> steps)
{
    // guard against empty routes, even though they shouldn't happen
    if (steps.empty())
        return steps;

    util::guidance::print(steps);
    BOOST_ASSERT(!hasTurnType(steps.back()) && hasWaypointType(steps.back()));
    for (auto prev = steps.begin(), itr = std::next(prev); itr != steps.end(); ++itr)
    {
        if (!hasTurnType(*itr))
            continue;

        if (hasTurnType(*itr, TurnType::NewName) && haveSameMode(*prev, *itr) && !hasLanes(*itr))
        {
            auto distance = itr->distance;
            const auto name = itr;

            // sum up all distances that can be relevant to the name change
            ++itr;
            while (!hasWaypointType(*itr) &&
                   (!hasTurnType(*itr) || hasTurnType(*itr, TurnType::Suppressed)) &&
                   distance < NAME_SEGMENT_CUTOFF_LENGTH)
            {
                distance += itr->distance;
                ++itr;
            }

            if (distance < NAME_SEGMENT_CUTOFF_LENGTH)
                name->maneuver.instruction.type = TurnType::Suppressed;

            prev = name;
        }
        else
        {
            // remember last item
            prev = itr;
        }
    }
    return steps;
}

} // namespace guidance
} // namespace engine
} // namespace osrm
