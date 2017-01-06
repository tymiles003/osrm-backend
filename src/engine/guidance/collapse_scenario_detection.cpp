#include "engine/guidance/collapse_scenario_detection.hpp"
#include "util/bearing.hpp"

#include <numeric>

#include <boost/assert.hpp>

namespace osrm
{
namespace engine
{
namespace guidance
{

namespace
{
bool bearingsAreReversed(const double bearing_in, const double bearing_out)
{
    // Nearly perfectly reversed angles have a difference close to 180 degrees (straight)
    const double left_turn_angle = [&]() {
        if (0 <= bearing_out && bearing_out <= bearing_in)
            return bearing_in - bearing_out;
        return bearing_in + 360 - bearing_out;
    }();
    return util::angularDeviation(left_turn_angle, 180) <= 35;
}

bool isLinkroad(const RouteStep &step)
{
    const constexpr double MAX_LINK_ROAD_LENGTH = 60.0;
    return step.distance <= MAX_LINK_ROAD_LENGTH && step.name_id == EMPTY_NAMEID;
}

bool noIntermediaryIntersections(const RouteStep &step)
{
    return std::all_of(step.intersections.begin() + 1,
                       step.intersections.end(),
                       [](const auto &intersection) { return intersection.entry.size() == 2; });
}

bool isCollapsableSegment(const RouteStep &step)
{
    const auto is_short = step.distance <= MAX_COLLAPSE_DISTANCE;
    return is_short && noIntermediaryIntersections(step);
}

} // namespace

bool basicCollapsePreconditions(const RouteStepIterator first, const RouteStepIterator second)
{
    const auto has_roundabout_type = hasRoundaboutType(first->maneuver.instruction) ||
                                     hasRoundaboutType(second->maneuver.instruction);

    return !has_roundabout_type && haveSameMode(*first, *second);
}

bool basicCollapsePreconditions(const RouteStepIterator first,
                                const RouteStepIterator second,
                                const RouteStepIterator third)
{
    const auto has_roundabout_type = hasRoundaboutType(first->maneuver.instruction) ||
                                     hasRoundaboutType(second->maneuver.instruction) ||
                                     hasRoundaboutType(third->maneuver.instruction);

    // require modes to match up
    return !has_roundabout_type && haveSameMode(*first, *second, *third);
}

bool isStaggeredIntersection(const RouteStepIterator step_prior_to_intersection,
                             const RouteStepIterator step_entering_intersection,
                             const RouteStepIterator step_leaving_intersection)
{
    BOOST_ASSERT(!hasWaypointType(*step_entering_intersection) &&
                 !(hasWaypointType(*step_leaving_intersection)));
    // don't touch roundabouts
    if (entersRoundabout(step_entering_intersection->maneuver.instruction) ||
        entersRoundabout(step_leaving_intersection->maneuver.instruction))
        return false;
    // Base decision on distance since the zig-zag is a visual clue.
    // If adjusted, make sure to check validity of the is_right/is_left classification below
    const constexpr auto MAX_STAGGERED_DISTANCE = 3; // debatable, but keep short to be on safe side

    const auto angle = [](const RouteStep &step) {
        const auto &intersection = step.intersections.front();
        const auto entry_bearing = intersection.bearings[intersection.in];
        const auto exit_bearing = intersection.bearings[intersection.out];
        return util::bearing::angleBetween(entry_bearing, exit_bearing);
    };

    // Instead of using turn modifiers (e.g. as in isRightTurn) we want to be more strict here.
    // We do not want to trigger e.g. on sharp uturn'ish turns or going straight "turns".
    // Therefore we use the turn angle to derive 90 degree'ish right / left turns.
    // This more closely resembles what we understand as Staggered Intersection.
    // We have to be careful in cases with larger MAX_STAGGERED_DISTANCE values. If the distance
    // gets large, sharper angles might be not obvious enough to consider them a staggered
    // intersection. We might need to consider making the decision here dependent on the actual turn
    // angle taken. To do so, we could scale the angle-limits by a factor depending on the distance
    // between the turns.
    const auto is_right = [](const double angle) { return angle > 45 && angle < 135; };
    const auto is_left = [](const double angle) { return angle > 225 && angle < 315; };

    const auto left_right =
        is_left(angle(*step_entering_intersection)) && is_right(angle(*step_leaving_intersection));
    const auto right_left =
        is_right(angle(*step_entering_intersection)) && is_left(angle(*step_leaving_intersection));

    // A RouteStep holds distance/duration from the maneuver to the subsequent step.
    // We are only interested in the distance between the first and the second.
    const auto is_short = step_entering_intersection->distance < MAX_STAGGERED_DISTANCE;
    const auto intermediary_mode_change =
        step_prior_to_intersection->mode == step_leaving_intersection->mode &&
        step_entering_intersection->mode != step_leaving_intersection->mode;

    const auto mode_change_when_entering =
        step_prior_to_intersection->mode != step_entering_intersection->mode;

    // previous step maneuver intersections should be length 1 to indicate that
    // there are no intersections between the two potentially collapsible turns
    return is_short && (left_right || right_left) && !intermediary_mode_change &&
           !mode_change_when_entering && noIntermediaryIntersections(*step_entering_intersection);
}

bool isUTurn(const RouteStepIterator step_prior_to_intersection,
             const RouteStepIterator step_entering_intersection,
             const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(
            step_prior_to_intersection, step_entering_intersection, step_leaving_intersection))
        return false;

    const bool takes_u_turn = bearingsAreReversed(
        util::bearing::reverse(step_entering_intersection->intersections.front()
                                   .bearings[step_entering_intersection->intersections.front().in]),
        step_leaving_intersection->intersections.front()
            .bearings[step_leaving_intersection->intersections.front().out]);

    if (!takes_u_turn)
        return false;

    // TODO check for name match after additional step
    const auto names_match = haveSameName(*step_prior_to_intersection, *step_leaving_intersection);

    if (!names_match)
        std::cout << "Failing due to name missmatch" << std::endl;
    if (!names_match)
        return false;

    const auto is_short = step_entering_intersection->distance <= MAX_COLLAPSE_DISTANCE;
    const auto only_allowed_turn = numberOfAllowedTurns(*step_leaving_intersection) == 1;

    return (is_short || isLinkroad(*step_entering_intersection) || only_allowed_turn) &&
           noIntermediaryIntersections(*step_entering_intersection);
}

bool isNameOszillation(const RouteStepIterator step_prior_to_intersection,
                       const RouteStepIterator step_entering_intersection,
                       const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(
            step_prior_to_intersection, step_entering_intersection, step_leaving_intersection))
        return false;

    const auto are_name_changes =
        (hasTurnType(*step_entering_intersection, TurnType::NewName) ||
         (hasTurnType(*step_entering_intersection, TurnType::Turn) &&
          hasModifier(*step_entering_intersection, DirectionModifier::Straight))) &&
        (hasTurnType(*step_leaving_intersection, TurnType::NewName) ||
         (hasTurnType(*step_leaving_intersection, TurnType::Suppressed) &&
          step_leaving_intersection->name_id == EMPTY_NAMEID) ||
         (hasTurnType(*step_leaving_intersection, TurnType::Turn) &&
          hasModifier(*step_leaving_intersection, DirectionModifier::Straight)));

    if (!are_name_changes)
        return false;

    const auto names_match =
        // accept empty names as well as same names
        step_prior_to_intersection->name_id == step_leaving_intersection->name_id ||
        haveSameName(*step_prior_to_intersection, *step_leaving_intersection);
    return names_match;
}

bool maneuverPreceededByNameChange(const RouteStepIterator step_prior_to_intersection,
                                   const RouteStepIterator step_entering_intersection,
                                   const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(
            step_prior_to_intersection, step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_collapsable = isCollapsableSegment(*step_entering_intersection);
    const auto is_name_change =
        hasTurnType(*step_entering_intersection, TurnType::NewName) ||
        ((hasTurnType(*step_entering_intersection, TurnType::Turn) ||
          hasTurnType(*step_entering_intersection, TurnType::Continue)) &&
         hasModifier(*step_entering_intersection, DirectionModifier::Straight));

    const auto followed_by_maneuver =
        hasTurnType(*step_leaving_intersection) &&
        !hasTurnType(*step_leaving_intersection, TurnType::Suppressed);

    return is_collapsable && is_name_change && followed_by_maneuver;
}

bool maneuverPreceededBySuppressedDirection(const RouteStepIterator step_entering_intersection,
                                            const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_collapsable = isCollapsableSegment(*step_entering_intersection);

    const auto is_suppressed_direction =
        hasTurnType(*step_entering_intersection, TurnType::Suppressed) &&
        !hasModifier(*step_entering_intersection, DirectionModifier::Straight);

    const auto followed_by_maneuver =
        hasTurnType(*step_leaving_intersection) &&
        !hasTurnType(*step_leaving_intersection, TurnType::Suppressed);

    const auto keeps_direction =
        areSameSide(*step_entering_intersection, *step_leaving_intersection);

    const auto has_choice = numberOfAllowedTurns(*step_entering_intersection) > 1;

    return is_collapsable && has_choice && is_suppressed_direction && followed_by_maneuver &&
           keeps_direction;
}

bool maneuverSucceededByNameChange(const RouteStepIterator step_entering_intersection,
                                   const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_collapsable = isCollapsableSegment(*step_entering_intersection);
    const auto followed_by_name_change =
        hasTurnType(*step_leaving_intersection, TurnType::NewName) ||
        ((hasTurnType(*step_leaving_intersection, TurnType::Turn) ||
          hasTurnType(*step_leaving_intersection, TurnType::Continue)) &&
         hasModifier(*step_leaving_intersection, DirectionModifier::Straight));

    const auto is_maneuver = hasTurnType(*step_entering_intersection) &&
                             !hasTurnType(*step_entering_intersection, TurnType::Suppressed);

    // a straight name change can overrule max collapse distance
    const auto is_strong_name_change =
        hasTurnType(*step_leaving_intersection, TurnType::NewName) &&
        hasModifier(*step_leaving_intersection, DirectionModifier::Straight) &&
        step_entering_intersection->distance <= 1.5 * MAX_COLLAPSE_DISTANCE;

    std::cout << "Check: " << is_collapsable << " " << followed_by_name_change << " " << is_maneuver
              << std::endl;

    return (is_collapsable || is_strong_name_change) && followed_by_name_change && is_maneuver;
}

bool maneuverSucceededBySuppressedDirection(const RouteStepIterator step_entering_intersection,
                                            const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_collapsable = isCollapsableSegment(*step_entering_intersection);

    const auto followed_by_suppressed_direction =
        hasTurnType(*step_leaving_intersection, TurnType::Suppressed) &&
        !hasModifier(*step_leaving_intersection, DirectionModifier::Straight);

    const auto is_maneuver = hasTurnType(*step_entering_intersection) &&
                             !hasTurnType(*step_entering_intersection, TurnType::Suppressed);

    const auto keeps_direction =
        areSameSide(*step_entering_intersection, *step_leaving_intersection);

    std::cout << "Check: " << is_collapsable << " " << followed_by_suppressed_direction << " "
              << is_maneuver << " " << keeps_direction << std::endl;

    return is_collapsable && followed_by_suppressed_direction && is_maneuver && keeps_direction;
}

bool nameChangeImmediatelyAfterSuppressed(const RouteStepIterator step_entering_intersection,
                                          const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto very_short = step_entering_intersection->distance < 0.25 * MAX_COLLAPSE_DISTANCE;
    const auto correct_types = hasTurnType(*step_entering_intersection, TurnType::Suppressed) &&
                               hasTurnType(*step_leaving_intersection, TurnType::NewName);

    return very_short && correct_types;
}

bool closeChoicelessTurnAfterTurn(const RouteStepIterator step_entering_intersection,
                                  const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_collapsable = isCollapsableSegment(*step_entering_intersection);
    const auto is_turn = !hasModifier(*step_entering_intersection, DirectionModifier::Straight);

    const auto followed_by_choiceless = numberOfAllowedTurns(*step_leaving_intersection) == 1;

    return is_collapsable && is_turn && followed_by_choiceless;
}

bool doubleChoiceless(const RouteStepIterator step_entering_intersection,
                      const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto double_choiceless =
        (numberOfAllowedTurns(*step_leaving_intersection) == 1) &&
        (step_entering_intersection->intersections.size() == 2) &&
        (std::count(step_entering_intersection->intersections.back().entry.begin(),
                    step_entering_intersection->intersections.back().entry.end(),
                    true) == 1);

    const auto short_enough = step_entering_intersection->distance < 1.5 * MAX_COLLAPSE_DISTANCE;

    return double_choiceless && short_enough;
}

bool straightTurnFollowedByChoiceless(const RouteStepIterator step_entering_intersection,
                                      const RouteStepIterator step_leaving_intersection)
{
    if (!basicCollapsePreconditions(step_entering_intersection, step_leaving_intersection))
        return false;

    const auto is_short = step_entering_intersection->distance <= 2 * MAX_COLLAPSE_DISTANCE;
    const auto has_correct_type = hasTurnType(*step_entering_intersection, TurnType::Continue) ||
                                  hasTurnType(*step_entering_intersection, TurnType::Turn);

    const auto is_straight = hasModifier(*step_entering_intersection, DirectionModifier::Straight);

    const auto only_choice = numberOfAllowedTurns(*step_leaving_intersection) == 1;

    return is_short && has_correct_type && is_straight && only_choice &&
           noIntermediaryIntersections(*step_entering_intersection);
}

} /* namespace guidance */
} /* namespace engine */
} /* namespace osrm */
