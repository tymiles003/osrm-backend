#ifndef ENGINE_HPP
#define ENGINE_HPP

#include "engine/api/match_parameters.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/api/tile_parameters.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/engine_config.hpp"
#include "engine/datafacade_provider.hpp"
#include "engine/plugins/match.hpp"
#include "engine/plugins/nearest.hpp"
#include "engine/plugins/table.hpp"
#include "engine/plugins/tile.hpp"
#include "engine/plugins/trip.hpp"
#include "engine/plugins/viaroute.hpp"
#include "engine/status.hpp"
#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/json_container.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace osrm
{
namespace engine
{

class EngineInterface
{
public:
    virtual Status Route(const api::RouteParameters &parameters, util::json::Object &result) const = 0;
    virtual Status Table(const api::TableParameters &parameters, util::json::Object &result) const = 0;
    virtual Status Nearest(const api::NearestParameters &parameters, util::json::Object &result) const = 0;
    virtual Status Trip(const api::TripParameters &parameters, util::json::Object &result) const = 0;
    virtual Status Match(const api::MatchParameters &parameters, util::json::Object &result) const = 0;
    virtual Status Tile(const api::TileParameters &parameters, std::string &result) const = 0;
};

template<typename AlgorithmT, template<typename A> class FacadeT>
class Engine final : public EngineInterface
{
  public:
    explicit Engine(const EngineConfig &config)
          : facade_provider(config.storage_config),
          route_plugin(config.max_locations_viaroute),       //
          table_plugin(config.max_locations_distance_table), //
          nearest_plugin(config.max_results_nearest),        //
          trip_plugin(config.max_locations_trip),            //
          match_plugin(config.max_locations_map_matching),   //
          tile_plugin()                                      //

    {
    }

    Engine(Engine &&) noexcept = delete;
    Engine &operator=(Engine &&) noexcept = delete;

    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;


    Status Route(const api::RouteParameters &params, util::json::Object &result) const
    {
        return route_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

    Status Table(const api::TableParameters &params, util::json::Object &result) const
    {
        return table_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

    Status Nearest(const api::NearestParameters &params, util::json::Object &result) const
    {
        return nearest_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

    Status Trip(const api::TripParameters &params, util::json::Object &result) const
    {
        return trip_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

    Status Match(const api::MatchParameters &params, util::json::Object &result) const
    {
        return match_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

    Status Tile(const api::TileParameters &params, std::string &result) const
    {
        return tile_plugin.HandleRequest(facade_provider.Get(), params, result);
    }

  private:
    DataFacadeProvider<AlgorithmT, FacadeT> facade_provider;

    const plugins::ViaRoutePlugin route_plugin;
    const plugins::TablePlugin table_plugin;
    const plugins::NearestPlugin nearest_plugin;
    const plugins::TripPlugin trip_plugin;
    const plugins::MatchPlugin match_plugin;
    const plugins::TilePlugin tile_plugin;
};
}
}

#endif // OSRM_IMPL_HPP
