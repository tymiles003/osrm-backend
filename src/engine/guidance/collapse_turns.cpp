#include "engine/guidance/collapse_turns.hpp"
#include "extractor/guidance/turn_instruction.hpp"
#include "engine/guidance/collapse_scenario_detection.hpp"
#include "engine/guidance/collapsing_utility.hpp"
#include "util/bearing.hpp"
#include "util/guidance/name_announcements.hpp"

#include "util/debug.hpp"

#include <cstddef>

#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <iterator>

using osrm::extractor::guidance::TurnInstruction;
using osrm::util::angularDeviation;
using namespace osrm::extractor::guidance;

namespace osrm
{
namespace engine
{
namespace guidance
{

using RouteSteps = std::vector<RouteStep>;
using RouteStepIterator = typename RouteSteps::iterator;
namespace
{
const constexpr double MAX_COLLAPSE_DISTANCE = 30;

// TODO rework this
double findTotalTurnAngle(const RouteStep &entry_step, const RouteStep &exit_step)
{
    const auto exit_intersection = exit_step.intersections.front();
    const auto exit_step_exit_bearing = exit_intersection.bearings[exit_intersection.out];
    const auto exit_step_entry_bearing =
        util::bearing::reverse(exit_intersection.bearings[exit_intersection.in]);

    const auto entry_intersection = entry_step.intersections.front();
    const auto entry_step_entry_bearing =
        util::bearing::reverse(entry_intersection.bearings[entry_intersection.in]);
    const auto entry_step_exit_bearing = entry_intersection.bearings[entry_intersection.out];

    const auto exit_angle =
        util::bearing::angleBetween(exit_step_entry_bearing, exit_step_exit_bearing);
    const auto entry_angle =
        util::bearing::angleBetween(entry_step_entry_bearing, entry_step_exit_bearing);

    const double total_angle =
        util::bearing::angleBetween(entry_step_entry_bearing, exit_step_exit_bearing);
    // We allow for minor deviations from a straight line
    if (((entry_step.distance < MAX_COLLAPSE_DISTANCE && exit_step.intersections.size() == 1) ||
         (entry_angle <= 185 && exit_angle <= 185) || (entry_angle >= 175 && exit_angle >= 175)) &&
        angularDeviation(total_angle, 180) > 20)
    {
        // both angles are in the same direction, the total turn gets increased
        // 
        // a ---- b
        //           \
        //              c
        //              |
        //              d
        //
        // Will be considered just like
        // 
        // a -----b
        //        |
        //        c
        //        |
        //        d
        return total_angle;
    }
    else
    {
        // to prevent ignoring angles like
        // 
        // a -- b
        //      |
        //      c -- d
        // 
        // We don't combine both turn angles here but keep the very first turn angle.
        // We choose the first one, since we consider the first maneuver in a merge range the
        // important one
        return entry_angle;
    }
}

inline void handleSliproad(RouteStepIterator sliproad_step)
{
    auto next_step = [&sliproad_step]() {
        auto next_step = findNextTurn(sliproad_step);
        while (isTrafficLightStep(*next_step))
        {
            // in sliproad checks, we should have made sure not to include invalid modes
            BOOST_ASSERT(haveSameMode(*sliproad_step, *next_step));
            sliproad_step->ElongateBy(*next_step);
            next_step->Invalidate();
            next_step = findNextTurn(next_step);
        }
        BOOST_ASSERT(haveSameMode(*sliproad_step, *next_step));
        return next_step;
    }();

    // have we reached the end?
    if (hasWaypointType(*next_step))
    {
        setInstructionType(*sliproad_step, TurnType::Turn);
    }
    else
    {
        const auto previous_step = findPreviousTurn(sliproad_step);
        const auto connecting_same_name_roads = haveSameName(*previous_step, *next_step);
        std::cout << "Found to be the same" << std::endl;
        auto sliproad_turn_type = connecting_same_name_roads ? TurnType::Continue : TurnType::Turn;
        setInstructionType(*sliproad_step, sliproad_turn_type);
        combineRouteSteps(*sliproad_step,
                          *next_step,
                          AdjustToCombinedTurnAngleStrategy(),
                          TransferSignageStrategy(),
                          TransferLanesStrategy());
    }
}

} // namespace

// STRATEGIES

// keep signage/other entries in route step intact
void NoModificationStrategy::operator()(RouteStep &, const RouteStep &) const
{
    // actually do nothing.
}

// transfer turn type from a different turn
void TransferTurnTypeStrategy::operator()(RouteStep &step_at_turn_location,
                                          const RouteStep &transfer_from_step) const
{
    step_at_turn_location.maneuver = transfer_from_step.maneuver;
}

void AdjustToCombinedTurnAngleStrategy::operator()(RouteStep &step_at_turn_location,
                                                   const RouteStep &transfer_from_step) const
{
    // TODO assert transfer_from_step == step_at_turn_location + 1
    const auto angle = findTotalTurnAngle(step_at_turn_location, transfer_from_step);
    step_at_turn_location.maneuver.instruction.direction_modifier = getTurnDirection(angle);
}

AdjustToCombinedTurnStrategy::AdjustToCombinedTurnStrategy(
    const RouteStep &step_prior_to_intersection)
    : step_prior_to_intersection(step_prior_to_intersection)
{
}

void AdjustToCombinedTurnStrategy::operator()(RouteStep &step_at_turn_location,
                                              const RouteStep &transfer_from_step) const
{
    const auto angle = findTotalTurnAngle(step_at_turn_location, transfer_from_step);
    const auto new_modifier = getTurnDirection(angle);

    const auto transferring_from_non_turn =
        hasTurnType(transfer_from_step, TurnType::NewName) ||
        (hasTurnType(transfer_from_step, TurnType::Turn) &&
         hasModifier(transfer_from_step, DirectionModifier::Straight)) ||
        (hasTurnType(transfer_from_step, TurnType::Continue) &&
         hasModifier(transfer_from_step, DirectionModifier::Straight));

    const auto maneuver_at_non_turn =
        hasTurnType(step_at_turn_location, TurnType::NewName) ||
        (hasTurnType(step_at_turn_location, TurnType::Turn) &&
         hasModifier(step_at_turn_location, DirectionModifier::Straight)) ||
        (hasTurnType(step_at_turn_location, TurnType::Continue) &&
         hasModifier(step_at_turn_location, DirectionModifier::Straight)) ||
        hasTurnType(step_at_turn_location, TurnType::Suppressed);

    if (transferring_from_non_turn || maneuver_at_non_turn)
    {
        std::cout << "New name: " << transfer_from_step.name
                  << " Before: " << step_prior_to_intersection.name << " "
                  << haveSameName(step_prior_to_intersection, transfer_from_step) << std::endl;
        if (hasTurnType(step_at_turn_location, TurnType::Suppressed))
        {
            std::cout << "A" << std::endl;
            if (new_modifier == DirectionModifier::Straight)
            {
                step_at_turn_location.maneuver.instruction = {TurnType::NewName, new_modifier};
            }
            else
            {
                step_at_turn_location.maneuver.instruction.type =
                    haveSameName(step_prior_to_intersection, transfer_from_step)
                        ? TurnType::Continue
                        : TurnType::Turn;
                step_at_turn_location.maneuver.instruction.direction_modifier = new_modifier;
            }
        }
        else if (hasTurnType(step_at_turn_location, TurnType::Continue) &&
                 !haveSameName(step_prior_to_intersection, transfer_from_step))
        {
            std::cout << "B" << std::endl;
            setInstructionType(step_at_turn_location, TurnType::Turn);
            step_at_turn_location.maneuver.instruction.direction_modifier = new_modifier;
        }
        else if (hasTurnType(step_at_turn_location, TurnType::Turn) &&
                 haveSameName(step_prior_to_intersection, transfer_from_step))
        {
            std::cout << "C" << std::endl;
            setInstructionType(step_at_turn_location, TurnType::Continue);
            step_at_turn_location.maneuver.instruction.direction_modifier = new_modifier;
        }
        else
        {
            std::cout << "No Case Triggered" << std::endl;
            step_at_turn_location.maneuver.instruction.direction_modifier = new_modifier;
        }
    }
    else
    {
        step_at_turn_location.maneuver.instruction.direction_modifier = new_modifier;
    }
}

StaggeredTurnStrategy::StaggeredTurnStrategy(const RouteStep &step_prior_to_intersection)
    : step_prior_to_intersection(step_prior_to_intersection)
{
}

void StaggeredTurnStrategy::operator()(RouteStep &step_at_turn_location,
                                       const RouteStep &transfer_from_step) const
{
    step_at_turn_location.maneuver.instruction.direction_modifier = DirectionModifier::Straight;
    step_at_turn_location.maneuver.instruction.type =
        haveSameName(step_prior_to_intersection, transfer_from_step) ? TurnType::Suppressed
                                                                     : TurnType::NewName;
}

SetFixedInstructionStrategy::SetFixedInstructionStrategy(
    const extractor::guidance::TurnInstruction instruction)
    : instruction(instruction)
{
}

void SetFixedInstructionStrategy::operator()(RouteStep &step_at_turn_location,
                                             const RouteStep &) const
{
    step_at_turn_location.maneuver.instruction = instruction;
}

void TransferSignageStrategy::operator()(RouteStep &step_at_turn_location,
                                         const RouteStep &transfer_from_step) const
{
    step_at_turn_location.AdaptStepSignage(transfer_from_step);
    step_at_turn_location.rotary_name = transfer_from_step.rotary_name;
    step_at_turn_location.rotary_pronunciation = transfer_from_step.rotary_pronunciation;
}

void TransferLanesStrategy::operator()(RouteStep &step_at_turn_location,
                                       const RouteStep &transfer_from_step) const
{
    step_at_turn_location.intersections.front().lanes =
        transfer_from_step.intersections.front().lanes;
    step_at_turn_location.intersections.front().lane_description =
        transfer_from_step.intersections.front().lane_description;
}

void suppressStep(RouteStep &step_at_turn_location, RouteStep &step_after_turn_location)
{
    return combineRouteSteps(step_at_turn_location,
                             step_after_turn_location,
                             NoModificationStrategy(),
                             NoModificationStrategy(),
                             NoModificationStrategy());
}

// OTHER IMPLEMENTATIONS
OSRM_ATTR_WARN_UNUSED
RouteSteps collapseTurnInstructions(RouteSteps steps)
{
    // make sure we can safely iterate over all steps (has depart/arrive with TurnType::NoTurn)
    BOOST_ASSERT(!hasTurnType(steps.front()) && !hasTurnType(steps.back()));
    BOOST_ASSERT(hasWaypointType(steps.front()) && hasWaypointType(steps.back()));

    if (steps.size() <= 2)
        return steps;

    std::cout << "Processing " << steps.size() << " steps." << std::endl;
    util::guidance::print(steps);
    // start of with no-op
    for (auto current_step = steps.begin() + 1; current_step + 1 != steps.end(); ++current_step)
    {
        std::cout << "At: " << std::distance(steps.begin(), current_step) << std::endl;
        if (entersRoundabout(current_step->maneuver.instruction) ||
            staysOnRoundabout(current_step->maneuver.instruction))
        {
            // Skip over all instructions within the roundabout
            for (; current_step + 1 != steps.end(); ++current_step)
                if (leavesRoundabout(current_step->maneuver.instruction))
                    break;

            // are we done for good?
            if (current_step + 1 == steps.end())
                break;
            else
                continue;
        }

        // only operate on actual turns
        if (!hasTurnType(*current_step))
            continue;

        // handle all situations involving the sliproad turn type
        if (hasTurnType(*current_step, TurnType::Sliproad))
        {
            handleSliproad(current_step);
            continue;
        }

        // don't collapse next step if it is a waypoint alread
        const auto next_step = findNextTurn(current_step);
        if (hasWaypointType(*next_step))
            break;

        const auto previous_step = findPreviousTurn(current_step);

        // don't collapse anything that does change modes
        if (current_step->mode != next_step->mode)
            continue;

        // handle staggered intersections:
        // a staggered intersection describes to turns in rapid succession that go in opposite
        // directions (e.g. right + left) with a very short segment in between
        if (isStaggeredIntersection(previous_step, current_step, next_step))
        {
            std::cout << "Staggered" << std::endl;
            combineRouteSteps(*current_step,
                              *next_step,
                              StaggeredTurnStrategy(*previous_step),
                              TransferSignageStrategy(),
                              NoModificationStrategy());
        }
        else if (isUTurn(previous_step, current_step, next_step))
        {
            std::cout << "Uturn" << std::endl;
            combineRouteSteps(
                *current_step,
                *next_step,
                SetFixedInstructionStrategy({TurnType::Continue, DirectionModifier::UTurn}),
                TransferSignageStrategy(),
                NoModificationStrategy());
        }
        else if (isNameOszillation(previous_step, current_step, next_step))
        {
            std::cout << "Name Oscillation" << std::endl;
            // first deactivate the second name switch
            suppressStep(*current_step, *next_step);
            // and then the first (to ensure both iterators to be valid)
            suppressStep(*previous_step, *current_step);
        }
        else if (maneuverPreceededByNameChange(previous_step, current_step, next_step))
        {
            const auto strategy = AdjustToCombinedTurnStrategy(*previous_step);
            strategy(*next_step, *current_step);
            // suppress previous step
            suppressStep(*previous_step, *current_step);
        }
        else if (maneuverSucceededByNameChange(current_step, next_step) ||
                 nameChangeImmediatelyAfterSuppressed(current_step, next_step) ||
                 maneuverSucceededBySuppressedDirection(current_step, next_step))
        {
            std::cout << "Name Change After" << std::endl;
            combineRouteSteps(*current_step,
                              *next_step,
                              AdjustToCombinedTurnStrategy(*previous_step),
                              TransferSignageStrategy(),
                              NoModificationStrategy());
        }
        else if (straightTurnFollowedByChoiceless(current_step, next_step) ||
                 maneuverPreceededBySuppressedDirection(current_step, next_step))
        {
            std::cout << "Straight By Choiceless" << std::endl;
            combineRouteSteps(*current_step,
                              *next_step,
                              AdjustToCombinedTurnStrategy(*previous_step),
                              TransferSignageStrategy(),
                              NoModificationStrategy());
        }
        else
        {
            std::cout << "No Collapse Scenario Triggers" << std::endl;
        }
    }
    return steps;
}

} // namespace guidance
} // namespace engine
} // namespace osrm
