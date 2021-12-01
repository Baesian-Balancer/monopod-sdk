
#ifndef SCENARIO_MONOPOD_WORLD_H
#define SCENARIO_MONOPOD_WORLD_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <real_time_tools/timer.hpp>
// #include <pybind11/stl.h>


class World
{
public:
    World();
    ~World();

    /**
     * Get the name of the world.
     *
     * @return The name of the world.
     */
    std::string name() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_MONOPOD_WORLD_H
