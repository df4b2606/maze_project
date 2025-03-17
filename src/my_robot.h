#ifndef __ROBOT_AGENT__H
#define __ROBOT_AGENT__H 

#include <string>
#include <math.h>
#include <vector>
#include <utility>
#include "enviro.h"

/**
 * @brief Global variable to track the current angle of the robot.
 */
double angle_cnt = 0;

/**
 * @brief Global variable to store the target angle for robot rotation.
 */
double angle_cnt_target = 0;

namespace {

    using namespace enviro;

    /**
     * @brief Struct to store the position coordinates of the robot.
     */
    struct Position {
        double x, y; ///< Coordinates of the position (x, y).

        /**
         * @brief Equality operator to compare two positions with a small tolerance.
         * @param other The other Position to compare against.
         * @return True if the positions are equal within a tolerance of 1e-5, false otherwise.
         */
        bool operator==(const Position& other) const {
            return std::abs(x - other.x) < 1e-5 && std::abs(y - other.y) < 1e-5;
        }
    };

    /// @brief Global vector to store positions visited by the robot.
    std::vector<Position> visited_positions;

    /**
     * @brief Checks if the robot has already visited a specific position.
     * @param x The x-coordinate of the position to check.
     * @param y The y-coordinate of the position to check.
     * @return True if the position has been visited, false otherwise.
     */
    bool has_visited(double x, double y) {
        Position current_pos = {x, y};
        for (const auto& pos : visited_positions) {
            if (pos == current_pos) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Records the current position of the robot in the visited positions list.
     * @param x The x-coordinate of the current position.
     * @param y The y-coordinate of the current position.
     */
    void record_position(double x, double y) {
        Position current_pos = {x, y};
        visited_positions.push_back(current_pos);
    }

    /**
     * @brief State class representing the destination state of the robot.
     * In this state, the robot stops moving as it has reached its target.
     */
    class Destination : public State, public AgentInterface {
    public:
        /**
         * @brief Called when entering the Destination state.
         * @param e The event that triggered the state transition.
         */
        void entry(const Event &e) {
            std::cout << "Entering Destination\n";
            track_velocity(0, 0);
        }      

        /**
         * @brief Called repeatedly while in the Destination state.
         * This state does nothing during its execution.
         */
        void during() {}  

        /**
         * @brief Called when exiting the Destination state.
         * @param e The event that triggered the state transition.
         */
        void exit(const Event &e) {}
    };

    /**
     * @brief State class for aligning the robot's angle to face the target directly.
     */
    class Aligning : public State, public AgentInterface {
    public:
        /**
         * @brief Called when entering the Aligning state.
         * @param e The event that triggered the state transition.
         */
        void entry(const Event &e) {
            std::cout << "Entering Aligning angle to find the object\n";
        }

        /**
         * @brief Called repeatedly while in the Aligning state.
         * Aligns the robot's angle until it matches the target angle, then transitions to the next state.
         */
        void during() {
            if (flag == 0) {
                flag = 1;
                angle_cnt_target += 1.5708; // Increment target angle by 90 degrees (pi/2 radians).
            }
            if (angle() <= angle_cnt_target) {
                track_velocity(0, 1.5708); // Rotate at angular velocity of pi/2 radians per second.
            }
            std::cout << angle() << "\n";
            std::cout << angle_cnt_target << "\n";

            if (angle() >= angle_cnt_target) {
                auto pos_agent = position();
                while (angle() != angle_cnt_target) {
                    teleport(pos_agent.x, pos_agent.y, angle_cnt_target); // Ensure precise angle alignment.
                }
                flag = 0;
                if (sensor_reflection_type(0) == "Target") {
                    angle_cnt_target = 0;
                    emit(Event("tick")); // Transition to next state if target is detected.
                }
            }
        }  

        /**
         * @brief Called when exiting the Aligning state.
         * @param e The event that triggered the state transition.
         */
        void exit(const Event &e) {}

    private:
        int flag = 0; ///< Flag to track whether angle alignment has started.
    };

    /**
     * @brief State class for moving the robot while detecting targets and walls.
     */
    class Moving : public State, public AgentInterface {
    public:
        /**
         * @brief Called when entering the Moving state.
         * Initializes the angle and elapsed time.
         * @param e The event that triggered the state transition.
         */
        void entry(const Event& e) {
            angle_cnt = angle();
            elapsed_time = 0.0;  // Reset elapsed time.
        }
        
        /**
         * @brief Called repeatedly while in the Moving state.
         * Moves the robot forward, adjusts speed based on sensor data, and transitions to other states as needed.
         */
        void during() {
            std::cout << "head sensor dist:" << sensor_value(0) << std::endl;
        
            // Parameter definitions
            double max_speed = 40.0;      // Maximum forward speed.
            double safety_dist = 60.0;    // Safety distance to obstacles.
            double acceleration = 60.0;   // Acceleration rate (speed per second).
        
            // Accumulate elapsed time (assuming delta_time() returns frame time step).
            elapsed_time += 0.016;  // Use fixed time step of 0.016 (60 FPS) if enviro.h does not provide delta_time().
        
            // Calculate base speed (accelerates over time).
            double v = 10 + acceleration * elapsed_time;
            v = std::min(v, max_speed);  // Cap speed at maximum.
        
            // Slow down if within safety distance.
            if (sensor_value(0) < safety_dist) {
                double distance_factor = sensor_value(0) / safety_dist;  // Distance ratio (0 to 1).
                v = distance_factor * max_speed;  // Adjust speed based on distance.
            }
        
            // Ensure speed is non-negative.
            v = std::max(5.0, v);
        
            // Apply velocity.
            track_velocity(v, 0);
        
            // Record current position.
            auto pos = position();
            record_position(pos.x, pos.y);
        
            // Collision or target detection.
            if (sensor_value(0) < 12) {
                if (sensor_reflection_type(0) == "Target") {
                    emit(Event("goal")); // Transition to Destination state if target is reached.
                }
                std::cout << "moving to turn state change since crash" << std::endl;
                emit(Event("tick")); // Transition to Turning state if obstacle is detected.
            }
            // Check all sensors for target detection.
            else if (sensor_reflection_type(0) == "Target" || sensor_reflection_type(1) == "Target" || 
                     sensor_reflection_type(2) == "Target" || sensor_reflection_type(3) == "Target") { 
                track_velocity(0, 0);  // Stop moving.
                if (sensor_reflection_type(0) == "Target") {
                    track_velocity(20, 0);  // Move forward if directly facing target.
                } else {
                    std::cout << "moving to turn state change since align angle to target" << std::endl;
                    emit(Event("adjust")); // Transition to Aligning state to face target.
                }
            }
        }
        
        /**
         * @brief Called when exiting the Moving state.
         * @param e The event that triggered the state transition.
         */
        void exit(const Event& e) {}
        
    private:
        double elapsed_time = 0.0;  ///< Tracks time spent in the Moving state.
    };

    /**
     * @brief State class for turning the robot to avoid obstacles.
     * Uses left and right sensors to determine the direction of open space.
     */
    class Turning : public State, public AgentInterface {
    public:
        /**
         * @brief Called when entering the Turning state.
         * @param e The event that triggered the state transition.
         */
        void entry(const Event& e) { }

        /**
         * @brief Called repeatedly while in the Turning state.
         * Turns the robot to face open space, then transitions back to Moving state.
         */
        void during() {
            if (flag == 0) {
                flag = 1;
                if (sensor_value(1) > sensor_value(3)) {
                    dir = 0; // Turn right if right sensor detects more open space.
                    angle_cnt += 1.5708; // Increment target angle by 90 degrees (pi/2 radians).
                } else {
                    dir = 1; // Turn left if left sensor detects more open space.
                    angle_cnt -= 1.5708; // Decrement target angle by 90 degrees (pi/2 radians).
                }
            }
            if ((dir == 0) && (angle() <= angle_cnt)) {
                track_velocity(0, 4); // Turn right at angular velocity of 4 radians per second.
            } else if ((angle() >= angle_cnt && (dir == 1))) {
                track_velocity(0, -4); // Turn left at angular velocity of -4 radians per second.
            }
            if ((dir == 0 && angle() >= angle_cnt) || (dir == 1 && angle() <= angle_cnt)) {
                auto pos = position();
                while (angle() != angle_cnt) {
                    teleport(pos.x, pos.y, angle_cnt); // Ensure precise angle alignment.
                }
                std::cout << "turn to moving state change" << std::endl;
                flag = 0;
                emit(Event("tick")); // Transition back to Moving state.
            }
        }

        /**
         * @brief Called when exiting the Turning state.
         * @param e The event that triggered the state transition.
         */
        void exit(const Event& e) {}

    private:
        int flag = 0; ///< Flag to track whether turning has started.
        int dir = 0;  ///< Direction of turning (0 for right, 1 for left).
    };

    /**
     * @brief State machine class for controlling the robot's behavior.
     * Manages transitions between Moving, Turning, Aligning, and Destination states.
     */
    class RobotController : public StateMachine, public AgentInterface {
    public:
        /**
         * @brief Constructs a RobotController and sets up state transitions.
         * Initial state is Moving.
         */
        RobotController() : StateMachine() {
            set_initial(moving);
            add_transition("tick", moving, turning);
            add_transition("tick", turning, moving);
            add_transition("tick", align_target_angle, moving);
            add_transition("adjust", moving, align_target_angle);
            add_transition("goal", moving, destination);
        }

        /**
         * @brief Calculates the closest distance to a wall detected by sensors.
         * @return The minimum distance to a wall, or a default large value (400.0) if no wall is detected.
         */
        double get_closest_wall_distance() {
            // Get distances from all sensors.
            double distances[4] = {
                sensor_value(0),  // Front
                sensor_value(1),  // Right
                sensor_value(2),  // Back
                sensor_value(3)   // Left
            };

            // Initialize minimum distance to a large value.
            double min_distance = std::numeric_limits<double>::max();

            // Find the minimum distance to a wall (excluding targets).
            for (int i = 0; i < 4; ++i) {
                if (sensor_reflection_type(i) != "Target" && distances[i] < min_distance) {
                    min_distance = distances[i];
                }
            }

            // Return a default large value if no wall is detected.
            if (min_distance == std::numeric_limits<double>::max()) {
                return 400.0;  // Assumes map size of 400; adjust as needed.
            }

            return min_distance;
        }

        Moving moving; ///< Moving state instance.
        Turning turning; ///< Turning state instance.
        Destination destination; ///< Destination state instance.
        Aligning align_target_angle; ///< Aligning state instance.
    };

    /**
     * @brief Agent class that encapsulates the robot controller.
     */
    class Controller : public Agent {
    public:
        /**
         * @brief Constructs a Controller agent and initializes the robot controller.
         * @param spec JSON specification for the agent.
         * @param world The simulation world in which the agent operates.
         */
        Controller(json spec, World& world) : Agent(spec, world) {
            add_process(c);
        }

        RobotController c; ///< Robot controller instance.
    };

    DECLARE_INTERFACE(Controller);

}

#endif