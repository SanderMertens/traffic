#include <traffic.h>
#include <iostream>

float randf(int n) {
    return static_cast<float>(rand() % n);
}

int main(int argc, char *argv[]) {
    flecs::log::set_level(-1);

    flecs::world world(argc, argv);

    ECS_IMPORT(world, FlecsScriptMath);

    world.import<flecs::components::transform>();
    world.import<flecs::components::graphics>();
    world.import<flecs::components::geometry>();
    world.import<flecs::components::gui>();
    world.import<flecs::components::physics>();
    world.import<flecs::components::input>();
    world.import<flecs::systems::transform>();
    world.import<flecs::systems::physics>();
    world.import<flecs::game>();
    world.import<flecs::systems::sokol>();

    world.import<traffic::cars>();

    world.script().filename("etc/assets/app.flecs").run();
    world.script().filename("etc/assets/main.flecs").run();

    // Populate lanes with cars
    int32_t count = 0;
    world.each([&](flecs::entity lane, traffic::Lane&) {
        if (lane.has<traffic::Corner>()) {
            return;
        }

        if (traffic::intersectionFromLane(lane)) {
            return;
        }

        // if (count) {
        //     return;
        // }

        // count ++;

        for (int i = 0; i >= 0; i --) {
            // if ((rand() % 3)) {
            //     continue;
            // }

            flecs::entity car = world.entity().child_of<traffic::car_root>()
                .set(traffic::Car{})
                .set(Box{3, 1, 1})
                .set(Emissive{1.0})
                .set(Color{1, 0, 1});
            traffic::addCarToLane(flecs::entity::null(), lane, car, i * 8 + randf(5));
        }
    });

    world.set_target_fps(60);
    // world.set_threads(8);

    return world.app()
        .enable_rest()
        .enable_stats()
        .target_fps(60)
        .delta_time(0.016)
        .run();
}
