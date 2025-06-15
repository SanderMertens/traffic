#pragma once
#include <flecs.h>

struct traffic {
    struct car_root {};
    struct road_root {};

    static constexpr int TrafficLightUpdateRate = 30;
    static constexpr int MaxCarsPerLane = 8;
    static constexpr float AccelerationForce = 1.5;
    static constexpr float BreakForce = 5;
    static constexpr float HardBreakForce = 12;
    static constexpr uint8_t MaxWaitCount = 60;

    static constexpr float PlaceholderCarMass = 10;
    static constexpr float PlaceholderCarLength = 3;

    enum Direction {
        Top = 0,
        Right = 1,
        Bottom = 2,
        Left = 3
    };

    enum Connection {
        TopToBottom = 0,
        LeftToRight = 1,
        TopToLeft = 2,
        TopToRight = 3,
        BottomToRight = 4,
        BottomToLeft = 5
    };

    struct Light {
        enum class Color {
            Red,
            Orange,
            Green
        };

        Color color = Color::Red;
        float t = 0;
        float cycle_time = 30.0;
        float red_pct = 0.75;
    };

    struct Car {
        enum class State {
            Unknown,
            Accelerating,
            Driving,
            Breaking,
            BreakingHard,
            Stopped,
            Crashed
        };

        enum class EndOfLaneState  {
            Default,
            ReserveIntersection,
            WaitForIntersection,
            MoveOnIntersection,
            OnIntersection
        };

        float position;
        float length;
        float mass;
        float speed;
        float target_speed;
        Car::State state;
        Car::EndOfLaneState eol_state;
        uint8_t reservation;
        uint8_t wait_count;
        flecs::entity_t next_lane;
    };

    struct LaneCars {
        LaneCars() {}

        Car cars[MaxCarsPerLane];

        Car& operator [](size_t index) {
            return cars[index];
        }

        const Car& operator [](size_t index) const {
            return cars[index];
        }

        int8_t count = 0;
    };

    struct LaneCarEntities {
        flecs::entity_t cars[MaxCarsPerLane];

        flecs::entity_t& operator [](size_t index) {
            return cars[index];
        }

        const flecs::entity_t& operator [](size_t index) const {
            return cars[index];
        }
    };

    struct Lane {
        Lane(float length_ = 0, float width_ = 0, float max_speed_ = 0, 
            flecs::entity_t next_ = 0, flecs::entity_t road_ = 0) 
        {
            length = length_;
            width = width_;
            max_speed = max_speed_;
            next = next_;
            road = road_;
        }

        float length;
        float width;
        float max_speed;
        flecs::entity_t next;
        flecs::entity_t road;
    };

    struct LaneTransform {
        mat4 value; // for transforming car positions on lane
    };

    struct Corner {
        float radius;
        bool invert_direction;
    };

    // Used to connect roads
    struct RoadConnect {
        flecs::entity_t road;
        int8_t edge; // which side of the road to connect to (0 or 1)
    };

    struct Road {
        Road(float length_ = 0, float lane_width_ = 0, float max_speed_ = 0, 
                bool corner_ = false, bool invert_corner_ = false, 
                RoadConnect next_ = {}, flecs::entity_t intersection = 0)
        {
            length = length_;
            lane_width = lane_width_;
            max_speed = max_speed_;
            corner = corner_;
            invert_corner = invert_corner_;
            next = next_;
        }

        float length;
        float lane_width;
        float max_speed;
        bool corner;
        bool invert_corner;
        RoadConnect next;
        flecs::entity_t intersection;
    };

    struct RoadLanes {
        flecs::entity_t lanes[2];

        flecs::entity_t& operator [](size_t index) {
            return lanes[index];
        }

        const flecs::entity_t& operator [](size_t index) const {
            return lanes[index];
        }
    };

    struct Intersection {
        // Incoming roads
        // 0: top
        // 1: right
        // 2: bottom
        // 3: left
        RoadConnect roads[4];
        float lane_width;
        float max_speed;
    };

    struct IntersectionRoads {
        // Roads on intersection
        // 0: top <-> bottom
        // 1: left <-> right
        // 2: top <-> left
        // 3: top <-> right
        // 4: bottom <-> right
        // 5: bottom <-> left
        flecs::entity_t roads[6];

        // Outgoing lane options (movements) for incoming lanes.
        // 0: top
        // 1: right
        // 2: bottom
        // 3: left
        flecs::entity_t movements[4];

        uint8_t current_reservation;
        uint8_t next_reservation;

        flecs::entity_t& operator [](size_t index) {
            return roads[index];
        }

        const flecs::entity_t& operator [](size_t index) const {
            return roads[index];
        }

        uint8_t reserve() {
            return next_reservation ++;
        }

        void release(uint8_t reservation) {
            assert(reservation == current_reservation);
            current_reservation ++;
        }
    };

    struct IntersectionMovement {
        // Backref to intersection
        flecs::entity_t intersection;

        // Outgoing lanes for incoming direction
        // 0: left
        // 1: straight
        // 2: right
        flecs::entity_t lanes[3];
    };

    static void addCarToLane(
        flecs::entity in_lane,
        flecs::entity out_lane,
        flecs::entity car, 
        float position = 0, 
        float speed = 0,
        float target_speed = 0,
        Car::State state = Car::State::Unknown,
        Car::EndOfLaneState eol_state = Car::EndOfLaneState::Default,
        uint8_t reservation = 0);
    

    static flecs::entity roadFromLane(
        flecs::entity lane);

    static flecs::entity intersectionFromLane(
        flecs::entity lane);

    traffic(flecs::world& world);
};
