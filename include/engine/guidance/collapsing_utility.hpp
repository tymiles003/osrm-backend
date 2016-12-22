#ifndef OSRM_ENGINE_GUIDANCE_COLLAPSING_UTILITY_HPP_
#define OSRM_ENGINE_GUIDANCE_COLLAPSING_UTILITY_HPP_

#include "extractor/guidance/turn_instruction.hpp"
#include "engine/guidance/route_step.hpp"
#include "util/guidance/name_announcements.hpp"

#include <cstddef>

using osrm::extractor::guidance::TurnInstruction;
using namespace osrm::extractor::guidance;

namespace osrm
{
namespace engine
{
namespace guidance
{

using RouteSteps = std::vector<RouteStep>;
using RouteStepIterator = typename RouteSteps::iterator;

// check if a step is completely without turn type
inline bool hasTurnType(const RouteStep &step)
{
    return step.maneuver.instruction.type != TurnType::NoTurn;
}
inline bool hasWaypointType(const RouteStep &step)
{
    return step.maneuver.waypoint_type != WaypointType::None;
}

//
inline RouteStepIterator findPreviousTurn(RouteStepIterator current_step)
{
    BOOST_ASSERT(!hasWaypointType(*current_step));
    // find the first element preceeding the current step that has an actual turn type (not
    // necessarily announced)
    do
    {
        // safety to do this loop is asserted in collapseTurnInstructions
        --current_step;
    } while (!hasTurnType(*current_step) && !hasWaypointType(*current_step));
    return current_step;
}

inline RouteStepIterator findNextTurn(RouteStepIterator current_step)
{
    BOOST_ASSERT(!hasWaypointType(*current_step));
    // find the first element preceeding the current step that has an actual turn type (not
    // necessarily announced)
    do
    {
        // safety to do this loop is asserted in collapseTurnInstructions
        ++current_step;
    } while (!hasTurnType(*current_step) && !hasWaypointType(*current_step));
    return current_step;
}

inline bool hasTurnType(const RouteStep &step, const TurnType::Enum type)
{
    return type == step.maneuver.instruction.type;
}
inline std::size_t numberOfAvailableTurns(const RouteStep &step)
{
    return step.intersections.front().entry.size();
}
inline std::size_t numberOfAllowedTurns(const RouteStep &step)
{
    return std::count(
        step.intersections.front().entry.begin(), step.intersections.front().entry.end(), true);
}

inline bool isTrafficLightStep(const RouteStep &step)
{
    return hasTurnType(step, TurnType::Suppressed) && numberOfAvailableTurns(step) == 2;
}

inline void setInstructionType(RouteStep &step, const TurnType::Enum type)
{
    step.maneuver.instruction.type = type;
}

inline bool haveSameMode(const RouteStep &lhs, const RouteStep &rhs)
{
    return lhs.mode == rhs.mode;
}

inline bool haveSameName(const RouteStep &lhs, const RouteStep &rhs)
{
    // make sure empty is not involved
    if (lhs.name_id == EMPTY_NAMEID || rhs.name_id == EMPTY_NAMEID)
        return false;

    // easy check to not go over the strings if not necessary
    else if (lhs.name_id == rhs.name_id)
        return true;

    // ok, bite the sour grape and check the strings already
    else
        return !util::guidance::requiresNameAnnounced(
            lhs.name, lhs.ref, lhs.pronunciation, rhs.name, rhs.ref, rhs.pronunciation);
}
} /* namespace guidance */
} /* namespace engine */
} /* namespace osrm */

#endif /* OSRM_ENGINE_GUIDANCE_COLLAPSING_UTILITY_HPP_ */
