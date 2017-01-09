#ifndef OSRM_ENGINE_DATAFACADE_PROVIDER_HPP
#define OSRM_ENGINE_DATAFACADE_PROVIDER_HPP

#include "engine/data_watchdog.hpp"
#include "engine/datafacade/process_memory_datafacade.hpp"
#include "engine/datafacade/shared_memory_datafacade.hpp"

#include "storage/shared_barriers.hpp"

namespace osrm
{
namespace engine
{

template<typename AlgorithmT, template<typename A> class FacadeT>
class DataFacadeProvider;

template<typename AlgorithmT>
class DataFacadeProvider<AlgorithmT, datafacade::SharedMemoryDataFacade>
{
    using FacadeT = datafacade::SharedMemoryDataFacade<AlgorithmT>;

public:
    DataFacadeProvider<FacadeT>(const storage::StorageConfig&) {}

    std::shared_ptr<FacadeT> Get() const
    {
        // FIXME we disregard the lock here
        return watchdog.GetDataFacade().second;
    }
private:
    // FIXME we won't need this after the new shared memory code landed
    mutable DataWatchdog<AlgorithmT> watchdog;
};

template<typename AlgorithmT>
class DataFacadeProvider<AlgorithmT, datafacade::ProcessMemoryDataFacade>
{
    using FacadeT = datafacade::ProcessMemoryDataFacade<AlgorithmT>;

public:
    DataFacadeProvider<FacadeT>(const storage::StorageConfig& config)
        : immutable_data_facade(std::make_shared<FacadeT>(config))
    {
    }

    std::shared_ptr<FacadeT> Get() const
    {
        return immutable_data_facade;
    }

private:
    std::shared_ptr<datafacade::ProcessMemoryDataFacade<AlgorithmT>> immutable_data_facade;
};

}
}

#endif
