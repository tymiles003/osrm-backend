#include "engine/guidance/verbosity_reduction.hpp"
#include "engine/guidance/collapsing_utility.hpp"

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

    BOOST_ASSERT(!hasTurnType(steps.back()) && hasWaypointType(steps.back()));
    for (auto prev = steps.begin(), itr = std::next(prev); itr != steps.end(); ++itr)
    {
        if (!hasTurnType(*itr) || hasTurnType(*itr, TurnType::Suppressed))
            continue;

        if (hasTurnType(*itr, TurnType::NewName) && haveSameMode(*prev, *itr) && !hasLanes(*itr))
        {
            std::cout << "Found Name: " << itr->name << " " << prev->name << std::endl;
            const auto name = itr;
            if (haveSameName(*prev, *itr))
            {
                std::cout << "Advancing nothing, suppressing name" << std::endl;
                name->maneuver.instruction.type = TurnType::Suppressed;
            }
            else
            {
                std::cout << "Names: " << prev->name << " " << itr->name << std::endl;
                auto distance = itr->distance;
                // sum up all distances that can be relevant to the name change
                while (
                    !hasWaypointType(*(itr + 1)) &&
                    (!hasTurnType(*(itr + 1)) || hasTurnType(*(itr + 1), TurnType::Suppressed)) &&
                    distance < NAME_SEGMENT_CUTOFF_LENGTH)
                {
                    ++itr;
                    distance += itr->distance;
                }

                if (distance < NAME_SEGMENT_CUTOFF_LENGTH)
                    name->maneuver.instruction.type = TurnType::Suppressed;
                else
                    prev = name;
            }
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
