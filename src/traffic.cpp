#include "traffic.h"
#include <algorithm>

using Position = flecs::components::transform::Position3;
using Rotation = flecs::components::transform::Rotation3;
using Scale = flecs::components::transform::Scale3;
using Transform = flecs::components::transform::Transform3;
using Color = flecs::components::graphics::Color;

float minDistanceForSpeed(float s1) {
    if (s1 < 0.1) {
        s1 = 0.1;
    }

    float r = s1 * 40;

    return r;
}

const char* carStateStr(traffic::Car::State state) {
    switch(state) {
    case traffic::Car::State::Unknown: return "Unknown";
    case traffic::Car::State::Accelerating: return "Accelerating";
    case traffic::Car::State::Driving: return "Driving";
    case traffic::Car::State::Breaking: return "Breaking";
    case traffic::Car::State::BreakingHard: return "BreakingHard";
    case traffic::Car::State::Stopped: return "Stopped";
    case traffic::Car::State::Crashed: return "Crashed";
    }
}

const char* carEndOfLaneStateStr(traffic::Car::EndOfLaneState state) {
    switch(state) {
    case traffic::Car::EndOfLaneState::Default: return "Default";
    case traffic::Car::EndOfLaneState::ReserveIntersection: return "ReserveIntersection";
    case traffic::Car::EndOfLaneState::WaitForIntersection: return "WaitForIntersection";
    case traffic::Car::EndOfLaneState::MoveOnIntersection: return "MoveOnIntersection";
    case traffic::Car::EndOfLaneState::OnIntersection: return "OnIntersection";
    }
}

traffic::traffic(flecs::world& world) {
    world.import<flecs::components::transform>();

    world.entity<road_root>("::roads");
    world.entity<car_root>("::cars");

    world.component<Light::Color>();

    world.component<Light>()
        .member("color", &Light::color)
        .member("t", &Light::t)
        .member("cycle_time", &Light::cycle_time)
        .member("red_pct", &Light::red_pct);

    world.component<Car>()
        .member("position", &Car::position)
        .member("length", &Car::length)
        .member("mass", &Car::mass)
        .member("speed", &Car::speed)
        .member("target_speed", &Car::target_speed)
        .member("state", &Car::state)
        .member("eol_state", &Car::eol_state)
        .member("reservation", &Car::reservation)
        .member("wait_count", &Car::wait_count)
        .member("next_lane", &Car::next_lane);

    world.component<LaneCars>()
        .member("cars", &LaneCars::cars)
        .member("count", &LaneCars::count);

    world.component<LaneCarEntities>()
        .member(flecs::Entity, "cars", MaxCarsPerLane);

    world.component<Lane>()
        .member("length", &Lane::length)
        .member("width", &Lane::width)
        .member("max_speed", &Lane::max_speed)
        .member(flecs::Entity, "next")
        .member(flecs::Entity, "road")
        .add(flecs::With, world.component<LaneCars>())
        .add(flecs::With, world.component<LaneCarEntities>());

    world.component<Corner>()
        .member("radius", &Corner::radius)
        .member("invert_direction", &Corner::invert_direction);

    world.component<RoadLanes>()
        .member("lanes", &RoadLanes::lanes);
    
    world.component<RoadConnect>()
        .member(flecs::Entity, "road")
        .member("edge", &RoadConnect::edge);

    world.component<Road>()
        .member("length", &Road::length)
        .member("lane_width", &Road::lane_width)
        .member("max_speed", &Road::max_speed)
        .member("corner", &Road::corner)
        .member("invert_corner", &Road::invert_corner)
        .member("next", &Road::next)
        .member(flecs::Entity, "intersection")
        .add(flecs::With, world.component<RoadLanes>());

    world.component<IntersectionRoads>()
        .member("roads", &IntersectionRoads::roads)
        .member("movements", &IntersectionRoads::movements)
        .member("current_reservation", &IntersectionRoads::current_reservation)
        .member("next_reservation", &IntersectionRoads::next_reservation);

    world.component<Intersection>()
        .member("roads", &Intersection::roads)
        .member("lane_width", &Intersection::lane_width)
        .member("max_speed", &Intersection::max_speed)
        .add(flecs::With, world.component<IntersectionRoads>());

    world.observer<Corner>()
        .event(flecs::OnSet)
        .each([](flecs::entity e, Corner& c) {
            Lane& l = e.ensure<Lane>();
            l.length = c.radius * GLM_PI * 0.5;
        });

    static auto oneShotSystem = [](flecs::iter& it) {
        while (it.next()) {
            it.each(); // forward to each callback
        }

        it.system().disable(); // run once
    };

    flecs::entity setLaneTransform = 
            world.system<const Road, const RoadLanes, Transform>("SetLaneTransform")
        .kind(flecs::PostUpdate)
        .immediate()
        .run(oneShotSystem, 
            [=](flecs::iter& it, size_t row, const Road& r, const RoadLanes& rl, Transform& m_road) 
        {
            flecs::world world = it.world();
            flecs::entity left = world.entity(rl[1]);
            flecs::entity right = world.entity(rl[0]);

            // Remove Transform component so lanes don't get automatically
            // transformed. Because lanes aren't children of their road, the
            // resulting matrix would be useless.
            left.remove<Transform>();
            right.remove<Transform>();

            LaneTransform& l_lt = left.ensure<LaneTransform>();
            LaneTransform& r_lt = right.ensure<LaneTransform>();

            const Position& l_p = left.get<Position>();
            const Position& r_p = right.get<Position>();

            const Rotation *l_r = left.try_get<Rotation>();
            const Rotation *r_r = right.try_get<Rotation>();                  

            glm_translate_to(m_road.value, *(vec3*)&l_p, l_lt.value);
            glm_translate_to(m_road.value, *(vec3*)&r_p, r_lt.value);
            
            if (l_r) {
                glm_rotate(l_lt.value, l_r->x, (vec3){1.0, 0.0, 0.0});
                glm_rotate(l_lt.value, l_r->y, (vec3){0.0, 1.0, 0.0});
                glm_rotate(l_lt.value, l_r->z, (vec3){0.0, 0.0, 1.0});
            }

            if (r_r) {
                glm_rotate(r_lt.value, r_r->x, (vec3){1.0, 0.0, 0.0});
                glm_rotate(r_lt.value, r_r->y, (vec3){0.0, 1.0, 0.0});
                glm_rotate(r_lt.value, r_r->z, (vec3){0.0, 0.0, 1.0});
            }
        });

    flecs::entity connectRoads = 
            world.system<const Road, const RoadLanes>("ConnectRoads")
        .immediate()
        .run(oneShotSystem, [](flecs::iter& it, size_t, const Road& r, const RoadLanes& rl) 
    {
            flecs::world world = it.world();
            flecs::entity next = world.entity(r.next.road);
            if (next) {
                int32_t edge = r.next.edge;
                const RoadLanes *rl_next = next.try_get<RoadLanes>();
                if (rl_next) {
                    Lane *left = 
                        world.entity(rl_next->lanes[edge]).try_get_mut<Lane>();
                    if (left) {
                        left->next = rl.lanes[edge];
                    }

                    Lane *right = 
                        world.entity(rl.lanes[edge - 1]).try_get_mut<Lane>();
                    if (right) {
                        right->next = rl_next->lanes[edge - 1];
                    }
                }
            }
        });

    world.observer<const Road, RoadLanes>("CreateRoad")
        .event(flecs::OnSet)
        .each([=](flecs::entity e, const Road& r, RoadLanes& rl) {
            flecs::world world = e.world();

            if (rl.lanes[0]) world.entity(rl.lanes[0]).destruct();
            if (rl.lanes[1]) world.entity(rl.lanes[1]).destruct();

            flecs::entity left = world.entity().child_of<road_root>();
            flecs::entity right = world.entity().child_of<road_root>();

            if (!r.corner) {
                left.set(Lane{r.length, r.lane_width, r.max_speed, 0, e})
                    .set(Position{0, 0, (r.lane_width / 2)})
                    .set(Rotation{0, (float)GLM_PI, 0});

                right.set(Lane{r.length, r.lane_width, r.max_speed, 0, e})
                     .set(Position{0, 0, -(r.lane_width / 2)});
            } else {
                float left_radius = r.lane_width * 1.5f;
                float left_speed = left_radius / 50;
                if (left_speed > r.max_speed) left_speed = r.max_speed;

                left.set(Lane{r.length, r.lane_width, left_speed, 0, e})
                    .set(Position{r.lane_width / 2, 0, r.lane_width / 2})
                    .set(Corner{ left_radius, true });

                float right_radius = r.lane_width / 2;
                float right_speed = right_radius / 30;
                if (right_speed > r.max_speed) right_speed = r.max_speed;

                right.set(Lane{r.length, r.lane_width, right_speed, 0, e})
                     .set(Position{-r.lane_width / 2, 0, -r.lane_width / 2})
                     .set(Corner{ right_radius });
            }

            if (r.invert_corner) {
                rl.lanes[0] = right;
                rl.lanes[1] = left;
            } else {
                rl.lanes[0] = left;
                rl.lanes[1] = right;
            }

            // ping one shot systems to run road initialization
            connectRoads.enable();
            setLaneTransform.enable();
        });

    flecs::entity connectIntersection = 
            world.system<const Intersection, IntersectionRoads>("ConnectIntersections")
        .immediate()
        .run(oneShotSystem,
            [](flecs::entity e, const Intersection& i, IntersectionRoads& ir) 
        {
            flecs::world world = e.world();

            // get connecting road
            auto road = [&](traffic::Direction d) {
                return world.entity(i.roads[d].road);
            };

            // get lane from connecting road
            auto road_lane = [&](traffic::Direction d, bool flip = false) {
                int8_t lane = i.roads[d].edge;
                if (flip) {
                    lane = 1 - lane;
                }
                return world.entity(road(d).get<RoadLanes>()[lane]);
            };

            // get lane from road on intersection
            auto i_lane = [&](traffic::Connection c, int8_t lane) {
                if (!ir[c]) {
                    return flecs::entity::null();
                }

                const auto& rl = world.entity(ir[c]).get<RoadLanes>();
                return world.entity(rl[lane]);
            };

            // Get movement for incoming lane. If there's only one available
            // don't create movement but connect it directly.
            auto i_movement = [&](Direction d, std::array<flecs::entity, 3> lanes) {
                flecs::entity_t lane = 0;
                int32_t count = 0;

                for (int i = 0; i < 3; i ++) {
                    if (lanes[i]) {
                        lane = lanes[i];
                        count ++;
                    }
                }

                if (count == 1) {
                    return lane;
                }

                auto r = ir.movements[d] = world.entity().child_of<road_root>()
                    .set(IntersectionMovement{e, 
                        {lanes[0], lanes[1], lanes[2]}});

                return r;
            };

            // Populate outgoing lanes for each incoming direction
            if (road(Top)) {
                road_lane(Top, true).get_mut<Lane>().next = 
                    i_movement(Top, {
                        i_lane(TopToRight, 0), 
                        i_lane(TopToBottom, 0), 
                        i_lane(TopToLeft, 0) 
                    });
            }

            if (road(Bottom)) {
                road_lane(Bottom, true).get_mut<Lane>().next = 
                    i_movement(Bottom, {
                        i_lane(BottomToLeft, 0),
                        i_lane(TopToBottom, 1), 
                        i_lane(BottomToRight, 0)
                    });
            }

            if (road(Left)) {
                road_lane(Left, true).get_mut<Lane>().next = 
                    i_movement(Left, {
                        i_lane(TopToLeft, 1),
                        i_lane(LeftToRight, 0), 
                        i_lane(BottomToLeft, 1)
                    });
            }

            if (road(Right)) {
                road_lane(Right, true).get_mut<Lane>().next = 
                    i_movement(Right, {
                        i_lane(BottomToRight, 1),
                        i_lane(LeftToRight, 1), 
                        i_lane(TopToRight, 1)
                    });
            }

            // Add traffic lights go incoming lanes
            // TODO

            // Connect movements to outgoing lanes
            if (road(Top) && road(Bottom)) {
                i_lane(TopToBottom, 0).get_mut<Lane>().next = road_lane(Bottom);
                i_lane(TopToBottom, 1).get_mut<Lane>().next = road_lane(Top);
            }

            if (road(Left) && road(Right)) {
                i_lane(LeftToRight, 0).get_mut<Lane>().next = road_lane(Right);
                i_lane(LeftToRight, 1).get_mut<Lane>().next = road_lane(Left);
            }

            if (road(Top) && road(Right)) {
                i_lane(TopToRight, 0).get_mut<Lane>().next = road_lane(Right);
                i_lane(TopToRight, 1).get_mut<Lane>().next = road_lane(Top);
            }

            if (road(Top) && road(Left)) {
                i_lane(TopToLeft, 0).get_mut<Lane>().next = road_lane(Left);
                i_lane(TopToLeft, 1).get_mut<Lane>().next = road_lane(Top);
            }

            if (road(Bottom) && road(Right)) {
                i_lane(BottomToRight, 0).get_mut<Lane>().next = road_lane(Right);
                i_lane(BottomToRight, 1).get_mut<Lane>().next = road_lane(Bottom);
            }

            if (road(Bottom) && road(Left)) {
                i_lane(BottomToLeft, 0).get_mut<Lane>().next = road_lane(Left);
                i_lane(BottomToLeft, 1).get_mut<Lane>().next = road_lane(Bottom);
            }
        });

    world.observer<const Intersection, IntersectionRoads>("CreateIntersection")
        .event(flecs::OnSet)
        .each([=](flecs::entity e, const Intersection& i, IntersectionRoads& ir) {
            flecs::world world = e.world();
            world.delete_with(flecs::ChildOf, e);

            // top <-> down
            if (i.roads[Top].road && i.roads[Bottom].road) {
                ir.roads[TopToBottom] = world.entity().child_of(e)
                    .set(Road{i.lane_width * 2, i.lane_width, i.max_speed, false})
                    .set(Position{0, 0, 0})
                    .set(Rotation{0, GLM_PI * 1.5, 0});
            }

            // left <-> right
            if (i.roads[Left].road && i.roads[Right].road) {
                ir.roads[LeftToRight] = world.entity().child_of(e)
                    .set(Road{i.lane_width * 2, i.lane_width, i.max_speed, false})
                    .set(Position{0, 0, 0})
                    .set(Rotation{0, GLM_PI, 0});
            }

            // top <-> left
            if (i.roads[Left].road && i.roads[Top].road) {
                ir.roads[TopToLeft] = world.entity().child_of(e)
                    .set(Road{i.lane_width, i.lane_width, i.max_speed, true, true})
                    .set(Position{0, 0, 0})
                    .set(Rotation{0, GLM_PI / 2, 0});
            }

            // top <-> right
            if (i.roads[Right].road && i.roads[Top].road) {
                ir.roads[TopToRight] = world.entity().child_of(e)
                    .set(Road{i.lane_width, i.lane_width, i.max_speed, true})
                    .set(Position{0, 0, 0})
                    .set(Rotation{0, GLM_PI, 0});
            }

            // bottom <-> right
            if (i.roads[Right].road && i.roads[Bottom].road) {
                ir.roads[BottomToRight] = world.entity().child_of(e)
                    .set(Road{i.lane_width, i.lane_width, i.max_speed, true, true})
                    .set(Position{0, 0, 0})
                    .set(Rotation{0, GLM_PI * 1.5, 0});
            }

            // bottom <-> left
            if (i.roads[Left].road && i.roads[Bottom].road) {
                ir.roads[BottomToLeft] = world.entity().child_of(e)
                    .set(Road{i.lane_width, i.lane_width, i.max_speed, true})
                    .set(Position{0, 0, 0});
            }

            // ping one shot systems to run intersection initialization
            connectIntersection.enable();
        });

    world.system<Light>("ProgressTrafficLights")
        .rate(TrafficLightUpdateRate)
        .each([](flecs::iter& it, size_t, Light& s) {
            float inc = (it.delta_time() * TrafficLightUpdateRate) / s.cycle_time;
            s.t += inc;

            if (s.t > 1.0) {
                s.t -= 1.0;
            }

            if (s.t < s.red_pct) {
                s.color = Light::Color::Red;
            } else if (s.t < 0.9) {
                s.color = Light::Color::Green;
            } else {
                s.color = Light::Color::Orange;
            }
        });

    world.system<LaneCars>("LaneProgressCars")
        .each([](LaneCars& cars) {
            for (int i = 0; i < MaxCarsPerLane; i ++) {
                Car& car = cars[i];
                car.position += car.speed;
            }
        });

    static int LaneTick = 0;

    world.system("IncrementLaneTick")
        .run([](flecs::iter& it) {
            LaneTick ++;
            if (LaneTick == 8) {
                LaneTick = 0;
            }
        });

    world.system<const Lane, LaneCars, const Light*>("LaneCarSetTargetSpeed")
        .each([](flecs::iter& it, size_t row, const Lane& lane, 
            LaneCars& cars, const Light* light) 
        {
            if (!cars.count) {
                // Lane is empty.
                return;
            }

            if ((it.entity(row).id() & 7) != LaneTick) {
                return;
            }

            flecs::world world = it.world();

            // Find target speed based on next car
            auto setTargetSpeed = [&](Car& car, float next_position, 
                float next_speed, Car::State next_state) 
            {
                // Crashed cars aren't allowed to do anything.
                if (car.state == Car::State::Crashed) {
                    return;
                }

                // Distance until next car.
                float distance = next_position - car.position;

                // Ideal minimum distance until next car based on speed (faster
                // means more distance).
                float min_d = minDistanceForSpeed(car.speed);

                // Find the target speed for the car.
                float target_speed = lane.max_speed;

                // If distance to next car is 0, we have a crash. Not good as
                // crashes *should* not happen.
                if (distance < 0) {
                    flecs::log::err("crash happened! (distance = %.2f, "
                        "position = %.2f, next_position = %.2f, speed = %.2f, "
                        "target_speed = %.2f)", distance, car.position, 
                            next_position, car.speed, car.target_speed);
                    car.speed = 0;
                    target_speed = 0;
                    car.state = Car::State::Crashed;
                    return;

                // If distance is smaller than ideal distance based on speed,
                // slow down the car.
                } else if (distance < min_d) {
                    if (next_state == Car::State::Breaking || 
                        next_state == Car::State::BreakingHard) 
                    {
                        // If the next car is breaking, set target speed to 0.
                        // This is the safest thing to do, since we don't know
                        // what the target speed for the next car is.
                        target_speed = 0;
                    } else {
                        // If the next car is accelerating or maintaining speed,
                        // try to follow with the same speed.
                        target_speed = next_speed;
                    }
                }

                // Obey the speed limit
                if (target_speed > lane.max_speed) {
                    target_speed = lane.max_speed;
                }

                // If we're waiting in front of an intersection and we haven't
                // gotten permission to enter yet it doesn't matter what the car
                // in front of us is doing. We need to stop.
                if (car.eol_state == Car::EndOfLaneState::WaitForIntersection || 
                    car.eol_state == Car::EndOfLaneState::ReserveIntersection) 
                {
                    target_speed = 0;
                }

                // Set target speed
                car.target_speed = target_speed;
            };

            // First car
            {
                Car& car = cars[0];

                // Traffic light handling
                if (light && light->color != Light::Color::Green) {
                    setTargetSpeed(car, lane.length, 0,
                        Car::State::BreakingHard);
                } else {
                    // Find car in next lane
                    flecs::entity next_lane = world.entity(lane.next);

                    // If intersection, lane.next doesn't point to a single Lane 
                    // but to all lanes a car could potentially take.
                    bool intersection = false;
                    const Lane *nl = next_lane.try_get<Lane>();
                    if (!nl) {
                        intersection = true;

                        // Use the lane that has been selected by the car.
                        if (car.next_lane) {
                            next_lane = world.entity(car.next_lane);
                            nl = next_lane.try_get<Lane>();
                            assert(nl != nullptr);
                        }    
                    }

                    float next_position_offset = 0;
                    const Car *next_car = nullptr;
                    if (nl) {
                        const LaneCars& nlc = next_lane.get<LaneCars>();
                        if (nlc.count) {
                            next_car = &nlc.cars[nlc.count - 1];                    
                        } else if (intersection) {
                            // Lookahead one lane if intersection. This makes sure the
                            // car doesn't go to fast if there's a short lane (like a
                            // right turn) with a car right behind it.
                            const LaneCars& nnlc = 
                                world.entity(nl->next).get<LaneCars>();
                            if (nnlc.count) {
                                next_car = &nnlc.cars[nnlc.count - 1];

                                // Add length of empty lane to get the right
                                // distance to the next car.
                                next_position_offset = nl->length;
                            }
                        }
                    }

                    if (next_car) {
                        // Compute position relative to current lane
                        float next_position = next_car->position - next_car->length / 2;
                        next_position += next_position_offset;
                        next_position += lane.length;

                        float next_speed = next_car->speed;
                        Car::State next_state = next_car->state;

                        setTargetSpeed(car, next_position, 
                            next_speed, next_state);
                    } else {
                        // There's nothing in front of us.
                        setTargetSpeed(car, 1000 * 1000, 1000 * 1000, 
                            Car::State::Driving);
                    }

                    if (nl) {
                        // If we're approaching the next lane, slow down to the
                        // max speed of the next lane.
                        float min_d = minDistanceForSpeed(car.speed);
                        if (lane.length - car.position < min_d) {
                            if (car.target_speed > nl->max_speed) {
                                car.target_speed = nl->max_speed;
                            }
                        }
                    }
                }

                if (car.target_speed == 0 && 
                    car.eol_state == Car::EndOfLaneState::MoveOnIntersection) 
                {
                    flecs::log::err(
                        "car is stopping while moving on intersection");
                }
            }

            // Remaining cars
            for (int i = 1; i < cars.count; i ++) {
                Car& car = cars[i];
                Car& next = cars[i - 1];

                float next_position = next.position - next.length;
                setTargetSpeed(car, next_position, next.speed, next.state);
            }
        });

    world.system<const Lane, LaneCars>("LaneSetEndOfLaneState")
        .each([](flecs::iter& it, size_t row, const Lane& lane, LaneCars& cars) {
            flecs::world world = it.world();
            flecs::entity next_lane = world.entity(lane.next);

            if ((it.entity(row).id() & 7) != LaneTick) {
                return;
            }

            auto find_next_lane = [&](const IntersectionMovement& im) {
                int8_t turn = rand() % 3;

                if (!im.lanes[turn]) {
                    int i;
                    for (i = 0; i < 2; i ++) {
                        if (im.lanes[(turn + i) % 3]) {
                            break;
                        }
                    }
                    if (i == 3) {
                        flecs::log::err("intersection without lanes :(");
                    } else {
                        turn = (turn + i) % 3;
                    }
                }

                return im.lanes[turn];
            };

            auto space_in_lane = [&](flecs::entity lane) {
                const LaneCars& cars = lane.get<LaneCars>();
                if (!cars.count) {
                    return lane.get<Lane>().length;
                }

                const Car& last_car = cars[cars.count - 1];
                return last_car.position - last_car.length;
            };

            if (cars.count) {
                // Only need to run logic for the first car. The rest of the 
                // cars will follow, and once the car moves onto the next lane
                // the next car becomes the first car.
                Car& car = cars[0];

                // Can only transition to end of lane state if we're in the
                // default state & we're near the end of the lane.
                if (car.eol_state == Car::EndOfLaneState::Default) {
                    float min_d = minDistanceForSpeed(car.speed);
                    if (min_d < 1) {
                        min_d = 1;
                    }

                    if ((lane.length - car.position) < min_d) {
                        // If the next lane isn't an ordinary lane, it's an
                        // intersection.
                        if (!next_lane.has<Lane>()) {
                            car.eol_state = 
                                Car::EndOfLaneState::ReserveIntersection;
                        }
                    }
                }

                switch(car.eol_state) {
                case Car::EndOfLaneState::Default:
                    car.wait_count = 0;
                    break;
                case Car::EndOfLaneState::ReserveIntersection: {
                    const auto& im = next_lane.get<IntersectionMovement>();
                    auto& ir = world.entity(im.intersection).get_mut<IntersectionRoads>();
                    car.reservation = ir.reserve();
                    car.next_lane = find_next_lane(im);
                    car.eol_state = Car::EndOfLaneState::WaitForIntersection;
                    car.target_speed = 0;
                    break;
                }

                case Car::EndOfLaneState::WaitForIntersection: {
                    const auto& im = next_lane.get<IntersectionMovement>();
                    auto& ir = world.entity(im.intersection).get_mut<IntersectionRoads>();

                    // It's our turn to move to the intersection
                    if (ir.current_reservation == car.reservation) {
                        // Check if destination lane is free. We don't want to
                        // move onto a lane that's not free yet as this could
                        // cause the car to block the intersection, which leads
                        // to gridlocks.
                        flecs::entity dst_lane = world.entity(car.next_lane);

                        // Lane should be empty, or we shouldn't have been 
                        // allowed on the intersection.
                        assert(dst_lane.get<LaneCars>().count == 0);

                        // Next lane is on intersection, we want to know whether
                        // the lane after that has enough space.
                        dst_lane = world.entity(dst_lane.get<Lane>().next);

                        if (space_in_lane(dst_lane) < car.length) {
                            // Can't move to destination lane, release intersection
                            ir.release(car.reservation);
                            car.eol_state = Car::EndOfLaneState::ReserveIntersection;
                        } else {
                            car.eol_state = Car::EndOfLaneState::MoveOnIntersection;
                        }
                    } else {
                        car.wait_count ++;
                        if (car.wait_count > MaxWaitCount) {
                            // If we're waiting too long try different lane. 
                            // This can prevent gridlocking where all incoming
                            // lanes are waiting on an outcoming lane that's
                            // blocked.
                            car.next_lane = find_next_lane(im);
                            car.wait_count = 0;
                        }
                    }
                    break;
                }
                case Car::EndOfLaneState::MoveOnIntersection:
                    break;
                case Car::EndOfLaneState::OnIntersection:
                    break;
                }
            }
        });

    world.system<const Lane, LaneCars, const Light*>("LaneCarSetDrivingState")
        .each([](flecs::iter& it, size_t row, const Lane& lane, 
            LaneCars& cars, const Light* light) 
        {
            if ((it.entity(row).id() & 7) != LaneTick) {
                return;
            }

            // Set driving behavior based on target speed
            auto setDrivingState = [](Car& car) {
                if (car.target_speed < car.speed) {
                    car.state = Car::State::Breaking;
                } else if (car.target_speed > car.speed) {
                    car.state = Car::State::Accelerating;
                } else {
                    car.state = Car::State::Driving;
                }

                if ((car.speed - car.target_speed) > 1) {
                    car.state = Car::State::BreakingHard;
                }
            };

            for (int i = 0; i < cars.count; i ++) {
                Car& car = cars[i];
                if (car.state == Car::State::Crashed) {
                    continue;
                }

                setDrivingState(car);
            }
        });

    world.system<LaneCars>("LaneAccelerateCars")
        .each([](flecs::iter& it, size_t row, LaneCars& cars) {
            float a = AccelerationForce * it.delta_system_time();
            float b = BreakForce * it.delta_system_time();
            float hb = HardBreakForce * it.delta_system_time();

            for (int i = 0; i < cars.count; i ++) {
                Car& car = cars[i];
                if (car.state == Car::State::Crashed) {
                    continue;
                }
                
                switch(car.state) {
                case Car::State::Accelerating:
                    assert(car.speed <= car.target_speed);

                    car.speed += a / car.mass;
                    if (car.speed > car.target_speed) {
                        car.speed = car.target_speed;
                        car.state = Car::State::Driving;
                    }
                    break;
                case Car::State::Breaking:
                    assert(car.speed >= car.target_speed);

                    car.speed -= b / car.mass;
                    if (car.speed < car.target_speed) {
                        car.speed = car.target_speed;
                        car.state = Car::State::Driving;
                    }
                    break;
                case Car::State::BreakingHard:
                    assert(car.speed >= car.target_speed);

                    car.speed -= hb / car.mass;
                    if (car.speed < car.target_speed) {
                        car.speed = car.target_speed;
                        car.state = Car::State::Driving;
                    }
                    break;
                default:
                    break;
                }

                if (car.speed <= 0) {
                    if (car.state != Car::State::Stopped) {
                        if (car.eol_state == Car::EndOfLaneState::OnIntersection) {
                            flecs::log::err("[%u] car stopped on intersection",
                                it.entity(row).get<LaneCarEntities>()[i]);
                        }
                    }

                    car.state = Car::State::Stopped;
                    car.speed = 0;
                }
            }
        });

    world.system<Lane, LaneCars, LaneCarEntities>("LaneMoveCarsToNextLane")
        .each([](flecs::iter& it, size_t row, Lane& lane, LaneCars& cars, LaneCarEntities& car_entities) {
            flecs::world world = it.world();

            // Find first car that's on the lane
            int i;
            for (i = 0; i < cars.count; i ++) {
                Car& car = cars[i];

                if (car.position < lane.length) {
                    break;
                }
            }

            // Move other cars to next lane
            if (i) {
                if (lane.next) {
                    for (int j = 0; j < i; j ++) {
                        Car& car = cars[j];
                        flecs::entity lane_next = world.entity(lane.next);
                        Car::EndOfLaneState eol = Car::EndOfLaneState::Default;

                        if (car.eol_state == Car::EndOfLaneState::MoveOnIntersection) {
                            // If moving on an intersection, lane.next points to
                            // all the different directions the incoming lane
                            // can take. The car has already selected the lane
                            // it wants to go on, so use that instead.
                            lane_next = world.entity(car.next_lane);

                            // Flag we're moving on intersection
                            eol = Car::EndOfLaneState::OnIntersection;
                        } else if (car.eol_state == Car::EndOfLaneState::OnIntersection) {
                            // Car was on intersection, and we're now moving off
                            // it. Release the intersection for the next car.
                            intersectionFromLane(it.entity(row))
                                .get_mut<IntersectionRoads>()
                                .release(car.reservation);
                        } else if (car.eol_state != Car::EndOfLaneState::Default) {
                            flecs::log::err(
                                "car %u doesn't have the right state to move on intersection",
                                car_entities[j]);
                        }

                        addCarToLane(
                            it.entity(row),
                            lane_next, 
                            world.entity(car_entities.cars[j]),
                            (car.position - lane.length),
                            car.speed,
                            car.target_speed,
                            car.state,
                            eol,
                            car.reservation);
                    }
                } else {
                    flecs::log::err("no lane to move to!");
                }

                // Move remaining cars up in the array
                std::copy(cars.cars + i, cars.cars + MaxCarsPerLane, cars.cars);
                std::copy(car_entities.cars + i, car_entities.cars + MaxCarsPerLane, car_entities.cars);

                cars.count -= i;
                assert(cars.count >= 0);
            }
        });

    world.system<const Lane, const LaneCars, const LaneCarEntities, 
            LaneTransform, const Corner*>("LaneUpdateCarEntities")
        .each([](flecs::iter& it, size_t row, 
            const Lane& lane, 
            const LaneCars& cars, 
            const LaneCarEntities& car_entities,
            LaneTransform& transform,
            const Corner *corner)
        {
            flecs::world world = it.world();

            for (int i = 0; i < cars.count; i ++) {
                const Car& car = cars[i];

                vec4 p = {0.0, 0.0, 0.0};
                float t = 0;

                if (!corner) {
                    p[0] = car.position - lane.length / 2.0;
                } else {
                    float pos = car.position;
                    if (corner->invert_direction) {
                        pos = lane.length - pos;
                    }
                    t = (pos / lane.length) * (GLM_PI / 2);
                    p[0] = sin(t) * corner->radius - corner->radius;
                    p[2] = cos(t) * corner->radius - corner->radius;
                }

                glm_mat4_mulv3(transform.value, p, 1.0f, p);

                flecs::entity e = world.entity(car_entities.cars[i])
                    .assign(car)
                    .assign(Position{
                        p[0], 
                        p[1] + 0.5f, 
                        p[2]});

                Rotation car_rotation = {};
                car_rotation.y = atan2(transform.value[0][2], transform.value[2][2]);
                if (corner) {
                    car_rotation.y += t;
                }

                e.assign(car_rotation);

                if (car.state == Car::State::Accelerating) {
                    e.assign(Color{0.4, 1, 0.1});
                } else if (car.state == Car::State::Breaking) {
                    e.assign(Color{1, 0.4, 0.1});
                } else if (car.state == Car::State::BreakingHard) {
                    e.assign(Color{1, 0.4, 0.1});
                } else if (car.state == Car::State::Stopped) {
                    e.assign(Color{1, 0.1, 0.0});
                } else if (car.state == Car::State::Crashed) {
                    e.assign(Color{0});
                } else {
                    e.assign(Color{0.4, 1, 0.1});
                }
            }
        });
}

void traffic::addCarToLane(
    flecs::entity in_lane, 
    flecs::entity out_lane, 
    flecs::entity car_entity, 
    float position, 
    float speed,
    float target_speed,
    Car::State state,
    Car::EndOfLaneState eol_state,
    uint8_t reservation) 
{
    if (!out_lane) {
        flecs::log::err("no lane to move to for car %u!", car_entity.id());
        return;
    }

    LaneCars *cars = out_lane.try_get_mut<traffic::LaneCars>();
    if (!cars) {
        flecs::log::err("lane is missing LaneCars component while moving car %u!", car_entity.id());
        return;
    }

    if (cars->count > MaxCarsPerLane) {
        flecs::log::err("lane already has max number of cars, can't move car %u", car_entity.id());
        return;
    }

    if (cars->count) {
        Car& last_car = (*cars)[cars->count - 1];

        if ((last_car.position - last_car.length) < position) {
            flecs::log::err("adding car %u to lane would cause a crash!", car_entity.id());
            position = 0;
            speed = 0;
            target_speed = 0;
        }
    }

    Car& car = (*cars)[cars->count];

    car.position = position;
    car.speed = speed;
    car.target_speed = target_speed;
    car.state = state;
    car.eol_state = eol_state;
    car.next_lane = out_lane;
    car.mass = PlaceholderCarMass;
    car.length = PlaceholderCarLength;
    car.reservation = reservation;

    LaneCarEntities& car_entities = out_lane.get_mut<LaneCarEntities>();
    car_entities[cars->count] = car_entity;

    cars->count ++;
    assert(cars->count < MaxCarsPerLane);
}

flecs::entity traffic::roadFromLane(flecs::entity lane) {
    flecs::world world = lane.world();
    return world.entity(lane.get<traffic::Lane>().road);
}

flecs::entity traffic::intersectionFromLane(flecs::entity lane) {
    flecs::world world = lane.world();
    return roadFromLane(lane).parent();
}
