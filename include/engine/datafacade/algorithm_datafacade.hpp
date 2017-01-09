#ifndef OSRM_ENGINE_DATAFACADE_ALGORITHM_DATAFACADE_HPP
#define OSRM_ENGINE_DATAFACADE_ALGORITHM_DATAFACADE_HPP

#include "engine/algorithm.hpp"

namespace osrm
{
namespace engine
{
namespace datafacade
{

template<typename AlgorithmT>
class AlgorithmDataFacade;

template<>
class AlgorithmDataFacade<algorithm::CH>
{
    /* TODO: Split CH-only part into this interface */
};

}
}
}

#endif
