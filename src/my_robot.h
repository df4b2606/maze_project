#ifndef __ROBOT_AGENT__H
#define __ROBOT_AGENT__H 

#include <string>
#include <math.h>
#include <vector>
#include <utility>
#include "enviro.h"

double angle_cnt = 0;
double angle_cnt_target = 0;

namespace {

    using namespace enviro;

    // 定义一个结构体来存储位置信息
    struct Position {
        double x, y;
        bool operator==(const Position& other) const {
            return std::abs(x - other.x) < 1e-5 && std::abs(y - other.y) < 1e-5;
        }
    };

    // 全局变量，用于存储机器人访问过的位置
    std::vector<Position> visited_positions;

    // 检查当前位置是否已经访问过
    bool has_visited(double x, double y) {
        Position current_pos = {x, y};
        for (const auto& pos : visited_positions) {
            if (pos == current_pos) {
                return true;
            }
        }
        return false;
    }

    // 记录当前位置
    void record_position(double x, double y) {
        Position current_pos = {x, y};
        visited_positions.push_back(current_pos);
    }

    // state find the goal
    class Goal : public State, public AgentInterface {
    public:
        void entry(const Event &e) {
            std::cout << "Entering Goal\n";
            track_velocity(0, 0);
        }      
        void during() {}  
        void exit(const Event &e) {}
    };

    // adjust the angle until the object direction is heading to target directly
    class Adjusting : public State, public AgentInterface {
    public:
        void entry(const Event &e) {
            std::cout << "Entering Adjusting angle to find the object\n";
        }
        void during() {
            if (flag == 0) {
                flag = 1;
                angle_cnt_target += 1.5708;
            }
            if (angle() <= angle_cnt_target) {
                track_velocity(0, 1.5708);
            }
            std::cout << angle() << "\n";
            std::cout << angle_cnt_target << "\n";

            if (angle() >= angle_cnt_target) {
                auto pos_agent = position();
                while (angle() != angle_cnt_target) {
                    teleport(pos_agent.x, pos_agent.y, angle_cnt_target);
                }
                flag = 0;
                if (sensor_reflection_type(0) == "Target") {
                    angle_cnt_target = 0;
                    emit(Event("tick"));
                }
            }
        }  
        void exit(const Event &e) {}
    private:
        int flag = 0;
    };

    // this is the state to moving forward and in the meanwhile detecting target and wall
    class MovingForward : public State, public AgentInterface {
        public:
            void entry(const Event& e) {
                angle_cnt = angle();
                elapsed_time = 0.0;  // 重置时间
            }
        
            void during() {
                std::cout << "head sensor dist:" << sensor_value(0) << std::endl;
        
                // 参数定义
                double max_speed = 40.0;      // 最大速度
                double safety_dist = 60.0;    // 安全距离
                double acceleration = 60.0;   // 加速度（单位：速度/秒，可调整）
        
                // 累加时间（假设 delta_time() 返回每帧时间步长）
                elapsed_time += 0.016;  // 如果 enviro.h 未提供此方法，可用固定步长替代，例如 0.016（60 FPS）
        
                // 计算基础速度（随时间加速）
                double v = 10+acceleration * elapsed_time;
                v = std::min(v, max_speed);  // 限制不超过最大速度
        
                // 如果小于安全距离，减速
                if (sensor_value(0) < safety_dist) {
                    double distance_factor = sensor_value(0) / safety_dist;  // 距离比例（0 到 1）
                    v = distance_factor * max_speed;  // 根据距离比例调整速度
                }
        
                // 确保速度非负
                v = std::max(5.0, v);
        
                // 应用速度
                track_velocity(v, 0);
        
                // 记录当前位置
                auto pos = position();
                record_position(pos.x, pos.y);
        
                // 碰撞或目标检测
                if (sensor_value(0) < 12) {
                    if (sensor_reflection_type(0) == "Target") {
                        emit(Event("goal"));
                    }
                    std::cout << "movingforward to rotate state change since crash" << std::endl;
                    emit(Event("tick"));
                }
                // 检查所有传感器是否检测到目标
                else if (sensor_reflection_type(0) == "Target" || sensor_reflection_type(1) == "Target" || 
                         sensor_reflection_type(2) == "Target" || sensor_reflection_type(3) == "Target") { 
                    track_velocity(0, 0);  // 停止移动
                    if (sensor_reflection_type(0) == "Target") {
                        track_velocity(20, 0);  // 如果正对目标，继续前进
                    } else {
                        std::cout << "movingforward to rotate state change since adjust angle to target" << std::endl;
                        emit(Event("adjust"));
                    }
                }
            }
            
            void exit(const Event& e) {}
        
        private:
            double elapsed_time = 0.0;  // 跟踪进入状态后的时间
        };
    // This state is used for rotating while wandering, before crashing move to open space compared with the left and right sensors
    class Rotating : public State, public AgentInterface {
    public:
        void entry(const Event& e) { }

        void during() {
            if (flag == 0) {
                flag = 1;
                if (sensor_value(1) > sensor_value(3)) {
                    dir = 0;
                    angle_cnt += 1.5708;
                } else {
                    dir = 1;
                    angle_cnt -= 1.5708;
                }
            }
            if ((dir == 0) && (angle() <= angle_cnt)) {
                track_velocity(0, 4);
            } else if ((angle() >= angle_cnt && (dir == 1))) {
                track_velocity(0, -4);
            }
            if ((dir == 0 && angle() >= angle_cnt) || (dir == 1 && angle() <= angle_cnt)) {
                auto pos = position();
                while (angle() != angle_cnt) {
                    teleport(pos.x, pos.y, angle_cnt);
                }
                std::cout << "rotate to movingforward state change" << std::endl;
                flag = 0;
                emit(Event("tick"));
            }
        }
        void exit(const Event& e) {}

    private:
        int flag = 0, dir = 0;
    };

    // Robot state machine
    class RobotController : public StateMachine, public AgentInterface {
    public:
        RobotController() : StateMachine() {
            set_initial(moving_forward);
            add_transition("tick", moving_forward, rotating);
            add_transition("tick", rotating, moving_forward);
            add_transition("tick", adjust_target_angle, moving_forward);
            add_transition("adjust", moving_forward, adjust_target_angle);
            add_transition("goal", moving_forward, goal);
        }

        // 返回与墙壁的最近距离
        double get_closest_wall_distance() {
            // 获取所有传感器的距离值
            double distances[4] = {
                sensor_value(0),  // 前方
                sensor_value(1),  // 右侧
                sensor_value(2),  // 后方
                sensor_value(3)   // 左侧
            };

            // 初始化最小距离为一个较大的值
            double min_distance = std::numeric_limits<double>::max();

            // 检查每个传感器，找到与墙壁的最小距离
            for (int i = 0; i < 4; ++i) {
                // 只考虑非目标的距离（假设 "Target" 是目标物体，不是墙壁）
                if (sensor_reflection_type(i) != "Target" && distances[i] < min_distance) {
                    min_distance = distances[i];
                }
            }

            // 如果没有检测到墙壁，返回一个默认大值（例如地图尺寸）
            if (min_distance == std::numeric_limits<double>::max()) {
                return 400.0;  // 假设地图大小为 400，可根据实际调整
            }

            return min_distance;
        }

        MovingForward moving_forward;
        Rotating rotating;
        Goal goal;
        Adjusting adjust_target_angle;
    };

    // The controller agent 
    class Controller : public Agent {
    public:
        Controller(json spec, World& world) : Agent(spec, world) {
            add_process(c);
        }

        RobotController c;
    };

    DECLARE_INTERFACE(Controller);

}

#endif