#ifndef TRIP_HPP
#define TRIP_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/trip_api.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/trip/trip_brute_force.hpp"
#include "engine/trip/trip_farthest_insertion.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/shortest_path.hpp"
#include "util/json_container.hpp"
#include "util/dist_table_wrapper.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{
namespace detail
{
// Object to hold all strongly connected components (scc) of a graph
// to access all graphs with component ID i, get the iterators by:
// auto start = std::begin(scc_component.component) + scc_component.range[i];
// auto end = std::begin(scc_component.component) + scc_component.range[i+1];
struct SCC_Component
{
    // in_component: all NodeIDs sorted by component ID
    // in_range: index where a new component starts
    //
    // example: NodeID 0, 1, 2, 4, 5 are in component 0
    //          NodeID 3, 6, 7, 8    are in component 1
    //          => in_component = [0, 1, 2, 4, 5, 3, 6, 7, 8]
    //          => in_range = [0, 5]
    SCC_Component(std::vector<NodeID> in_component_nodes, std::vector<size_t> in_range)
        : component(std::move(in_component_nodes)), range(std::move(in_range))
    {
        BOOST_ASSERT_MSG(component.size() > 0, "there's no scc component");
        BOOST_ASSERT_MSG(*std::max_element(range.begin(), range.end()) == component.size(),
                         "scc component ranges are out of bound");
        BOOST_ASSERT_MSG(*std::min_element(range.begin(), range.end()) == 0,
                         "invalid scc component range");
        BOOST_ASSERT_MSG(std::is_sorted(std::begin(range), std::end(range)),
                         "invalid component ranges");
    }

    std::size_t GetNumberOfComponents() const
    {
        BOOST_ASSERT_MSG(range.size() > 0, "there's no range");
        return range.size() - 1;
    }

    const std::vector<NodeID> component;
    std::vector<std::size_t> range;
};

SCC_Component SplitUnaccessibleLocations(const std::size_t number_of_locations,
                                         const util::DistTableWrapper<EdgeWeight> &result_table);

}

template <typename AlgorithmT, template <typename A> class FacadeT>
class TripPlugin final : public BasePlugin<AlgorithmT, FacadeT>
{
  private:
    using SuperT = BasePlugin<AlgorithmT, FacadeT>;

    mutable SearchEngineData heaps;
    mutable routing_algorithms::ShortestPathRouting<AlgorithmT, FacadeT> shortest_path;
    mutable routing_algorithms::ManyToManyRouting<AlgorithmT, FacadeT> duration_table;
    const int max_locations_trip;

    InternalRouteResult ComputeRoute(const FacadeT<AlgorithmT> &facade,
                                     const std::vector<PhantomNode> &phantom_node_list,
                                     const std::vector<NodeID> &trip) const;

  public:
    explicit TripPlugin(const int max_locations_trip_)
        : shortest_path(heaps), duration_table(heaps), max_locations_trip(max_locations_trip_)
    {
    }

    Status HandleRequest(const std::shared_ptr<FacadeT<AlgorithmT>> facade,
                         const api::TripParameters &parameters,
                         util::json::Object &json_result) const;
};



template <typename AlgorithmT, template <typename A> class FacadeT>
InternalRouteResult
TripPlugin<AlgorithmT, FacadeT>::ComputeRoute(const FacadeT<AlgorithmT> &facade,
                                              const std::vector<PhantomNode> &snapped_phantoms,
                                              const std::vector<NodeID> &trip) const
{
    InternalRouteResult min_route;
    // given he final trip, compute total duration and return the route and location permutation
    PhantomNodes viapoint;
    const auto start = std::begin(trip);
    const auto end = std::end(trip);
    // computes a roundtrip from the nodes in trip
    for (auto it = start; it != end; ++it)
    {
        const auto from_node = *it;
        // if from_node is the last node, compute the route from the last to the first location
        const auto to_node = std::next(it) != end ? *std::next(it) : *start;

        viapoint = PhantomNodes{snapped_phantoms[from_node], snapped_phantoms[to_node]};
        min_route.segment_end_coordinates.emplace_back(viapoint);
    }
    BOOST_ASSERT(min_route.segment_end_coordinates.size() == trip.size());

    shortest_path(facade, min_route.segment_end_coordinates, {false}, min_route);

    BOOST_ASSERT_MSG(min_route.shortest_path_length < INVALID_EDGE_WEIGHT, "unroutable route");
    return min_route;
}

template <typename AlgorithmT, template <typename A> class FacadeT>
Status TripPlugin<AlgorithmT, FacadeT>::HandleRequest(
    const std::shared_ptr<FacadeT<AlgorithmT>> facade,
    const api::TripParameters &parameters,
    util::json::Object &json_result) const
{
    BOOST_ASSERT(parameters.IsValid());

    // enforce maximum number of locations for performance reasons
    if (max_locations_trip > 0 &&
        static_cast<int>(parameters.coordinates.size()) > max_locations_trip)
    {
        return SuperT::Error("TooBig", "Too many trip coordinates", json_result);
    }

    if (!SuperT::CheckAllCoordinates(parameters.coordinates))
    {
        return SuperT::Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    auto phantom_node_pairs = SuperT::GetPhantomNodes(*facade, parameters);
    if (phantom_node_pairs.size() != parameters.coordinates.size())
    {
        return SuperT::Error("NoSegment",
                     std::string("Could not find a matching segment for coordinate ") +
                         std::to_string(phantom_node_pairs.size()),
                     json_result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == parameters.coordinates.size());

    auto snapped_phantoms = SuperT::SnapPhantomNodes(phantom_node_pairs);

    const auto number_of_locations = snapped_phantoms.size();

    // compute the duration table of all phantom nodes
    const auto result_table = util::DistTableWrapper<EdgeWeight>(
        duration_table(*facade, snapped_phantoms, {}, {}), number_of_locations);

    if (result_table.size() == 0)
    {
        return Status::Error;
    }

    const constexpr std::size_t BF_MAX_FEASABLE = 10;
    BOOST_ASSERT_MSG(result_table.size() == number_of_locations * number_of_locations,
                     "Distance Table has wrong size");

    // get scc components
    detail::SCC_Component scc = detail::SplitUnaccessibleLocations(number_of_locations, result_table);

    std::vector<std::vector<NodeID>> trips;
    trips.reserve(scc.GetNumberOfComponents());
    // run Trip computation for every SCC
    for (std::size_t k = 0; k < scc.GetNumberOfComponents(); ++k)
    {
        const auto component_size = scc.range[k + 1] - scc.range[k];

        BOOST_ASSERT_MSG(component_size > 0, "invalid component size");

        std::vector<NodeID> scc_route;
        auto route_begin = std::begin(scc.component) + scc.range[k];
        auto route_end = std::begin(scc.component) + scc.range[k + 1];

        if (component_size > 1)
        {

            if (component_size < BF_MAX_FEASABLE)
            {
                scc_route =
                    trip::BruteForceTrip(route_begin, route_end, number_of_locations, result_table);
            }
            else
            {
                scc_route = trip::FarthestInsertionTrip(
                    route_begin, route_end, number_of_locations, result_table);
            }
        }
        else
        {
            scc_route = std::vector<NodeID>(route_begin, route_end);
        }

        trips.push_back(std::move(scc_route));
    }
    if (trips.empty())
    {
        return SuperT::Error("NoTrips", "Cannot find trips", json_result);
    }

    // compute all round trip routes
    std::vector<InternalRouteResult> routes;
    routes.reserve(trips.size());
    for (const auto &trip : trips)
    {
        routes.push_back(ComputeRoute(*facade, snapped_phantoms, trip));
    }

    api::TripAPI trip_api{*facade, parameters};
    trip_api.MakeResponse(trips, routes, snapped_phantoms, json_result);

    return Status::Ok;
}
}
}
}

#endif // TRIP_HPP
