#include "engine/plugins/trip.hpp"
#include "extractor/tarjan_scc.hpp"
#include "util/dist_table_wrapper.hpp" // to access the dist table more easily
#include "util/matrix_graph_wrapper.hpp" // wrapper to use tarjan scc on dist table

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <memory>
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
// takes the number of locations and its duration matrix,
// identifies and splits the graph in its strongly connected components (scc)
// and returns an SCC_Component
SCC_Component SplitUnaccessibleLocations(const std::size_t number_of_locations,
                                         const util::DistTableWrapper<EdgeWeight> &result_table)
{

    if (std::find(std::begin(result_table), std::end(result_table), INVALID_EDGE_WEIGHT) ==
        std::end(result_table))
    {
        // whole graph is one scc
        std::vector<NodeID> location_ids(number_of_locations);
        std::iota(std::begin(location_ids), std::end(location_ids), 0);
        std::vector<size_t> range = {0, location_ids.size()};
        return SCC_Component(std::move(location_ids), std::move(range));
    }

    // Run TarjanSCC
    auto wrapper = std::make_shared<util::MatrixGraphWrapper<EdgeWeight>>(result_table.GetTable(),
                                                                          number_of_locations);
    auto scc = extractor::TarjanSCC<util::MatrixGraphWrapper<EdgeWeight>>(wrapper);
    scc.Run();

    const auto number_of_components = scc.GetNumberOfComponents();

    std::vector<std::size_t> range_insertion;
    std::vector<std::size_t> range;
    range_insertion.reserve(number_of_components);
    range.reserve(number_of_components);

    std::vector<NodeID> components(number_of_locations, 0);

    std::size_t prefix = 0;
    for (std::size_t j = 0; j < number_of_components; ++j)
    {
        range_insertion.push_back(prefix);
        range.push_back(prefix);
        prefix += scc.GetComponentSize(j);
    }
    // senitel
    range.push_back(components.size());

    for (std::size_t i = 0; i < number_of_locations; ++i)
    {
        components[range_insertion[scc.GetComponentID(i)]] = i;
        ++range_insertion[scc.GetComponentID(i)];
    }

    return SCC_Component(std::move(components), std::move(range));
}

}
}
}
}
