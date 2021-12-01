#include "myproject/test.h"


class World::Impl
{
public:
    struct
    {
        std::string worldName;
    } buffers;
};

World::World()
    : pImpl{std::make_unique<Impl>()}
{
    // Set up the names for world and model.
    pImpl->buffers.worldName  = "real_world";
}

World::~World() = default;

std::string World::name() const
{
    return pImpl->buffers.worldName;
}
