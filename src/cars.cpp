#include "traffic.h"
#include <algorithm>

using Position = flecs::components::transform::Position3;
using Rotation = flecs::components::transform::Rotation3;
using Scale = flecs::components::transform::Scale3;
using Transform = flecs::components::transform::Transform3;
using Color = flecs::components::graphics::Color;

namespace traffic {

float minDistanceForSpeed(float s1) {
    if (s1 < 0.1) {
        s1 = 0.1;
    }

    float r = s1 * 40;

    return r;
}

const char* carStateStr(Car::State state) {
    switch(state) {
    case Car::State::Unknown: return "Unknown";
    case Car::State::Accelerating: return "Accelerating";
    case Car::State::Driving: return "Driving";
    case Car::State::Breaking: return "Breaking";
    case Car::State::BreakingHard: return "BreakingHard";
    case Car::State::Stopped: return "Stopped";
    case Car::State::Crashed: return "Crashed";
    }
}

const char* carEndOfLaneStateStr(Car::EndOfLaneState state) {
    switch(state) {
    case Car::EndOfLaneState::Default: return "Default";
    case Car::EndOfLaneState::ReserveIntersection: return "ReserveIntersection";
    case Car::EndOfLaneState::WaitForIntersection: return "WaitForIntersection";
    case Car::EndOfLaneState::MoveOnIntersection: return "MoveOnIntersection";
    case Car::EndOfLaneState::OnIntersection: return "OnIntersection";
    }
}

cars::cars(flecs::world& world) {
    world.import<flecs::components::transform>();

    // Scopes for storing road and car entities
    world.entity<road_root>("::roads");
    world.entity<car_root>("::cars");

    // Component registration. This makes sure that components can be inspected
    // with the explorer & serialized to JSON.

    world.component<Light::Color>();

    world.component<Light>()
        .member("color", &Light::color)
        .member("t", &Light::t)
        .member("cycle_time", &Light::cycle_time)
        .member("red_pct", &Light::red_pct);

    // Specifying the path here so that the components get registered in the 
    // scope of Car vs. in the scope of the module.
    world.component<Car::State>("Car::State");
    world.component<Car::EndOfLaneState>("Car::EndOfLaneState");

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

    // Utility for systems that should only run when requested
    static auto oneShotSystem = [](flecs::iter& it) {
        while (it.next()) {
            it.each(); // forward to each callback
        }

        it.system().disable(); // run once
    };

    // Observer that sets the length of a corner lane according to the radius
    world.observer<Corner>()
        .event(flecs::OnSet)
        .each([](flecs::entity e, Corner& c) {
            Lane& l = e.ensure<Lane>();
            l.length = c.radius * GLM_PI * 0.5;
        });

    // System that sets the lane transform. This is used to transform car 
    // entities that are on the lane and ensures cars have the right position
    // and rotation.
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

    // Connect two roads together.
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

    // Observer that creates a road. A road is a container for two (or in the 
    // future, more) lanes, one for each direction.
    world.observer<const Road, RoadLanes>("CreateRoad")
        .event(flecs::OnSet)
        .each([=](flecs::entity e, const Road& r, RoadLanes& rl) {
            flecs::world world = e.world();

            if (rl.lanes[0]) world.entity(rl.lanes[0]).destruct();
            if (rl.lanes[1]) world.entity(rl.lanes[1]).destruct();

            flecs::entity left = world.entity().child_of<road_root>();
            flecs::entity right = world.entity().child_of<road_root>();

            if (!r.corner) {
                // Create a regular road
                left.set(Lane{r.length, r.lane_width, r.max_speed, 0, e})
                    .set(Position{0, 0, (r.lane_width / 2)})
                    .set(Rotation{0, (float)GLM_PI, 0});

                right.set(Lane{r.length, r.lane_width, r.max_speed, 0, e})
                     .set(Position{0, 0, -(r.lane_width / 2)});
            } else {
                // Create a corner road. Set the max speed of corner roads 
                // according to the radius, so that cars will slow down when
                // doing a sharp turn.

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

    // Connect in and outgoing roads (lanes) on intersection. This system will
    // create the necessary connections based on whether this is a two-way, a
    // three-way or a four-way intersection.
    flecs::entity connectIntersection = 
            world.system<const Intersection, IntersectionRoads>("ConnectIntersections")
        .immediate()
        .run(oneShotSystem,
            [](flecs::entity e, const Intersection& i, IntersectionRoads& ir) 
        {
            flecs::world world = e.world();

            // Get connecting road
            auto road = [&](Direction d) {
                return world.entity(i.roads[d].road);
            };

            // Get lane from connecting road
            auto roadLane = [&](Direction d, bool flip = false) {
                int8_t lane = i.roads[d].edge;
                if (flip) {
                    lane = 1 - lane;
                }
                return world.entity(road(d).get<RoadLanes>()[lane]);
            };

            // Get lane from road on intersection
            auto intersectionLane = [&](Connection c, int8_t lane) {
                if (!ir[c]) {
                    return flecs::entity::null();
                }

                const auto& rl = world.entity(ir[c]).get<RoadLanes>();
                return world.entity(rl[lane]);
            };

            // Get movement for incoming lane. If there's only one available
            // don't create movement but connect it directly.
            auto intersectionMovement = [&](Direction d, std::array<flecs::entity, 3> lanes) {
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
                roadLane(Top, true).get_mut<Lane>().next = 
                    intersectionMovement(Top, {
                        intersectionLane(TopToRight, 0), 
                        intersectionLane(TopToBottom, 0), 
                        intersectionLane(TopToLeft, 0) 
                    });
            }

            if (road(Bottom)) {
                roadLane(Bottom, true).get_mut<Lane>().next = 
                    intersectionMovement(Bottom, {
                        intersectionLane(BottomToLeft, 0),
                        intersectionLane(TopToBottom, 1), 
                        intersectionLane(BottomToRight, 0)
                    });
            }

            if (road(Left)) {
                roadLane(Left, true).get_mut<Lane>().next = 
                    intersectionMovement(Left, {
                        intersectionLane(TopToLeft, 1),
                        intersectionLane(LeftToRight, 0), 
                        intersectionLane(BottomToLeft, 1)
                    });
            }

            if (road(Right)) {
                roadLane(Right, true).get_mut<Lane>().next = 
                    intersectionMovement(Right, {
                        intersectionLane(BottomToRight, 1),
                        intersectionLane(LeftToRight, 1), 
                        intersectionLane(TopToRight, 1)
                    });
            }

            // Add traffic lights to incoming lanes
            // TODO

            // Connect movements to outgoing lanes
            if (road(Top) && road(Bottom)) {
                intersectionLane(TopToBottom, 0).get_mut<Lane>().next = roadLane(Bottom);
                intersectionLane(TopToBottom, 1).get_mut<Lane>().next = roadLane(Top);
            }

            if (road(Left) && road(Right)) {
                intersectionLane(LeftToRight, 0).get_mut<Lane>().next = roadLane(Right);
                intersectionLane(LeftToRight, 1).get_mut<Lane>().next = roadLane(Left);
            }

            if (road(Top) && road(Right)) {
                intersectionLane(TopToRight, 0).get_mut<Lane>().next = roadLane(Right);
                intersectionLane(TopToRight, 1).get_mut<Lane>().next = roadLane(Top);
            }

            if (road(Top) && road(Left)) {
                intersectionLane(TopToLeft, 0).get_mut<Lane>().next = roadLane(Left);
                intersectionLane(TopToLeft, 1).get_mut<Lane>().next = roadLane(Top);
            }

            if (road(Bottom) && road(Right)) {
                intersectionLane(BottomToRight, 0).get_mut<Lane>().next = roadLane(Right);
                intersectionLane(BottomToRight, 1).get_mut<Lane>().next = roadLane(Bottom);
            }

            if (road(Bottom) && road(Left)) {
                intersectionLane(BottomToLeft, 0).get_mut<Lane>().next = roadLane(Left);
                intersectionLane(BottomToLeft, 1).get_mut<Lane>().next = roadLane(Bottom);
            }
        });

    // Create intersection. An intersection is a container for at least two and
    // at most four roads.
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

    // Cars move "on rails", meaning they only have a single float value to 
    // express their position on their current lane.
    world.system<LaneCars>("LaneProgressCars")
        .each([](LaneCars& cars) {
            for (int i = 0; i < MaxCarsPerLane; i ++) {
                Car& car = cars[i];
                car.position += car.speed;
            }
        });

    // Performance optimization: instead of processing each lane every tick, 
    // each lane is processed every 8 ticks. The LaneTick is used by systems 
    // that progress the lanes to determine which lane should be processed in
    // the current tick.
    static int LaneTick = 0;

    world.system("IncrementLaneTick")
        .run([](flecs::iter& it) {
            LaneTick ++;
            if (LaneTick == 8) {
                LaneTick = 0;
            }
        });

    // The main system that determines the behavior of cars. Cars are "on rails"
    // meaning they can only move along the direction of the current lane 
    // they're on. As a result, the individual behavior of the cars is 
    // exclusively defined by their speed.
    world.system<const Lane, LaneCars, const Light*>("LaneCarSetTargetSpeed")
        .each([](flecs::iter& it, size_t row, const Lane& lane, 
            LaneCars& cars, const Light* light) 
        {
            if (!cars.count) {
                // Lane is empty.
                return;
            }

            // Determine if lane should be processed this tick
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

            // The first car needs additional handling since it needs to 
            // lookahead to the next lane to see what's coming. The cars behind
            // it just follow the car in front of them.
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

                // This should never happen since we check whether there's space
                // in the lane after the intersection before moving on the 
                // intersection. If this were to happen it can gridlock the
                // simulation.
                if (car.target_speed == 0 && 
                    car.eol_state == Car::EndOfLaneState::MoveOnIntersection) 
                {
                    flecs::log::err(
                        "car is stopping while moving on intersection");
                }
            }

            // Remaining cars. Calls the same setTargetSpeed function but just
            // with the car that's in front.
            for (int i = 1; i < cars.count; i ++) {
                Car& car = cars[i];
                Car& next = cars[i - 1];

                float next_position = next.position - next.length;
                setTargetSpeed(car, next_position, next.speed, next.state);
            }
        });

    // This system sets the state of the first car in the lane based on what the
    // lane is connected to. If the lane is connected to just another lane, 
    // nothing special needs to happen. If the lane is connected to an 
    // intersection, the car has to pick a direction and reserve the 
    // intersection. Future traffic light handling will also go here.
    world.system<const Lane, LaneCars>("LaneSetEndOfLaneState")
        .each([](flecs::iter& it, size_t row, const Lane& lane, LaneCars& cars) {
            if (!cars.count) {
                return;
            }

            flecs::world world = it.world();
            flecs::entity next_lane = world.entity(lane.next);
            
            // Determine if lane should be processed this tick
            if ((it.entity(row).id() & 7) != LaneTick) {
                return;
            }

            // Select a lane to move to when approaching an intersection.
            auto findNextLane = [&](const IntersectionMovement& im) {
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

            // Find out how much space is left in the lane.
            auto spaceInLane = [&](flecs::entity lane) {
                const LaneCars& cars = lane.get<LaneCars>();
                if (!cars.count) {
                    return lane.get<Lane>().length;
                }

                const Car& last_car = cars[cars.count - 1];
                return last_car.position - last_car.length;
            };

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
            
            // Default end of lane state: used when moving into another lane.
            case Car::EndOfLaneState::Default:
                car.wait_count = 0;
                break;

            // We're approaching an intersection, reserve it and select the
            // lane we want to move into.
            case Car::EndOfLaneState::ReserveIntersection: {
                const auto& im = next_lane.get<IntersectionMovement>();
                auto& ir = world.entity(im.intersection).get_mut<IntersectionRoads>();
                car.reservation = ir.reserve();
                car.next_lane = findNextLane(im);
                car.eol_state = Car::EndOfLaneState::WaitForIntersection;
                car.target_speed = 0;
                break;
            }

            // We've created a reservation for the intersection. Now we need to
            // wait until we acquire it.
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

                    if (spaceInLane(dst_lane) < car.length) {
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
                        car.next_lane = findNextLane(im);
                        car.wait_count = 0;
                    }
                }
                break;
            }

            // We're moving on the intersection. Nothing needs to be done here,
            // when we enter the next lane the state will get updated to
            // OnIntersection.
            case Car::EndOfLaneState::MoveOnIntersection:
                break;

            // We're on the intersection. Nothing needs to be done here, as the
            // state will reset to Default once we move off the intersection.
            case Car::EndOfLaneState::OnIntersection:
                break;
            }
        });

    // Set driving behavior based on target speed
    world.system<const Lane, LaneCars, const Light*>("LaneCarSetDrivingState")
        .each([](flecs::iter& it, size_t row, const Lane& lane, 
            LaneCars& cars, const Light* light) 
        {
            // Determine if lane should be processed this tick
            if ((it.entity(row).id() & 7) != LaneTick) {
                return;
            }

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
                            // This should never happen since we check whether
                            // there's space in the lane after the intersection
                            // before we move on the intersection. If this 
                            // happens it could cause a gridlock.
                            flecs::log::err("[%u] car stopped on intersection",
                                it.entity(row).get<LaneCarEntities>()[i]);
                        }
                    }

                    car.state = Car::State::Stopped;
                    car.speed = 0;
                }
            }
        });

    // System that finds all cars that have moved beyond the end of the lane,
    // and moves the car to the next lane.
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
                            // This could happen if a car didn't start breaking
                            // soon enough and entered the intersection without
                            // a reservation. Shouldn't happen as this could 
                            // cause cars to crash.
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

    // For efficiency reasons the simulation works exclusively on car arrays on
    // the Lane component. To actually visualize the cars however we need 
    // entities. This system updates the entities associated with the cars in
    // the simulation with the most up to date position and rotation.
    // Without this system the simulation would still run, but the cars would 
    // not appear to be moving. It would theoretically be possible to only 
    // update the car entities that are visible.
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
                    // For normal lanes cars simply follow a straight line.
                    p[0] = car.position - lane.length / 2.0;
                } else {
                    // For corner lanes cars trace a quarter of a circle.
                    float pos = car.position;
                    if (corner->invert_direction) {
                        pos = lane.length - pos;
                    }
                    t = (pos / lane.length) * (GLM_PI / 2);
                    p[0] = sin(t) * corner->radius - corner->radius;
                    p[2] = cos(t) * corner->radius - corner->radius;
                }

                // Get actual position by multiplying the car position with the
                // lane transform
                glm_mat4_mulv3(transform.value, p, 1.0f, p);

                // Get the car rotation from the lane transform matrix, add it
                // to any rotation from being on a corner.
                Rotation car_rotation = {};
                car_rotation.y = atan2(transform.value[0][2], transform.value[2][2]);
                if (corner) {
                    car_rotation.y += t;
                }

                flecs::entity e = world.entity(car_entities.cars[i])
                    .assign(car)
                    .assign(Position{ p[0], p[1] + 0.5f, p[2] })
                    .assign(car_rotation);

                // Assign a color based on the car state.
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

void addCarToLane(
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

    LaneCars *cars = out_lane.try_get_mut<LaneCars>();
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

flecs::entity roadFromLane(flecs::entity lane) {
    flecs::world world = lane.world();
    return world.entity(lane.get<Lane>().road);
}

flecs::entity intersectionFromLane(flecs::entity lane) {
    flecs::world world = lane.world();
    return roadFromLane(lane).parent();
}

}
