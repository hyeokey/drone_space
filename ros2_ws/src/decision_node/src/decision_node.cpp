#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode() : Node("decision_node") {
        RCLCPP_INFO(get_logger(), "DecisionNode started");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&DecisionNode::localPosCb, this, std::placeholders::_1));

        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            std::bind(&DecisionNode::statusCb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&DecisionNode::loop, this));
    }

private:
    enum class State {
        INIT,
        OFFBOARD,
        ARMING,
        TAKEOFF,
        HOVER,
        GOTO_TARGET,
        TARGET_HOVER,
        LAND,
        FINISHED
    };

    State state_ = State::INIT;

    const double target_lat_ = 47.397214;
    const double target_lon_ = 8.552146;
    const float target_alt_ = 5.0f;   // meters

    float x_{0}, y_{0}, z_{0};
    uint8_t nav_state_{0}, arm_state_{0};

    double ref_lat_{0}, ref_lon_{0};
    bool ref_ready_{false};

    float target_x_{0}, target_y_{0};

    rclcpp::Time state_start_;
    rclcpp::Time last_hover_log_;

    int offboard_counter_{0};

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void localPosCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;

        if (!ref_ready_ && msg->ref_timestamp > 0) {
            ref_lat_ = msg->ref_lat;
            ref_lon_ = msg->ref_lon;
            ref_ready_ = true;
            computeTargetLocal();
        }
    }

    void statusCb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        arm_state_ = msg->arming_state;
    }

    void computeTargetLocal() {
        constexpr double R = 6371000.0;
        double dlat = (target_lat_ - ref_lat_) * M_PI / 180.0;
        double dlon = (target_lon_ - ref_lon_) * M_PI / 180.0;

        target_x_ = dlat * R;
        target_y_ = dlon * R * cos(ref_lat_ * M_PI / 180.0);

        RCLCPP_INFO(get_logger(),
            "[TARGET] Local X=%.2f Y=%.2f", target_x_, target_y_);
    }

    void publishOffboard() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = now_us();
        offboard_pub_->publish(msg);
    }

    void publishSetpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = now_us();
        msg.position = {x, y, z};
        traj_pub_->publish(msg);
    }

    void sendCmd(uint16_t cmd, float p1 = 0, float p2 = 0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now_us();
        cmd_pub_->publish(msg);
    }

    uint64_t now_us() { return get_clock()->now().nanoseconds() / 1000; }
    rclcpp::Time now() { return get_clock()->now(); }

    void loop() {
        publishOffboard();

        switch (state_) {
        case State::INIT:
            publishSetpoint(0, 0, -target_alt_);
            if (++offboard_counter_ > 20) {
                RCLCPP_INFO(get_logger(), "[STATE] OFFBOARD");
                state_ = State::OFFBOARD;
                offboard_counter_ = 0;
            }
            break;

        case State::OFFBOARD:
            publishSetpoint(0, 0, -target_alt_);
            sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                RCLCPP_INFO(get_logger(), "[STATE] ARMING");
                state_ = State::ARMING;
            }
            break;

        case State::ARMING:
            publishSetpoint(0, 0, -target_alt_);
            sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
            if (arm_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(get_logger(), "[STATE] TAKEOFF");
                state_ = State::TAKEOFF;
            }
            break;

        case State::TAKEOFF:
            publishSetpoint(0, 0, -target_alt_);
            // 목표 고도 도달 여부 확인 (0.1m 허용)
            if (std::abs(z_ + target_alt_) < 0.1f) {
                RCLCPP_INFO(get_logger(), "[STATE] HOVER");
                state_ = State::HOVER;
                state_start_ = now();
                last_hover_log_ = now();
            } else {
                // TAKEOFF 중에는 z값만 로그
                RCLCPP_INFO(get_logger(),
                    "[TAKEOFF] z=%.2f", z_);
            }
            break;

        case State::HOVER:
            publishSetpoint(0, 0, -target_alt_);
            if ((now() - last_hover_log_).seconds() >= 0.5) {
                last_hover_log_ = now();
                RCLCPP_INFO(get_logger(),
                    "[HOVER] t=%.1f x=%.2f y=%.2f z=%.2f",
                    (now() - state_start_).seconds(),
                    x_, y_, z_);
            }
            if ((now() - state_start_).seconds() > 10.0) {
                RCLCPP_INFO(get_logger(), "[STATE] GOTO_TARGET");
                state_ = State::GOTO_TARGET;
            }
            break;

        case State::GOTO_TARGET:
            publishSetpoint(target_x_, target_y_, -target_alt_);
            if (dist2D() < 1.0) {
                RCLCPP_INFO(get_logger(), "[STATE] TARGET_HOVER");
                state_ = State::TARGET_HOVER;
                state_start_ = now();
                last_hover_log_ = now();
            }
            break;

        case State::TARGET_HOVER:
            publishSetpoint(target_x_, target_y_, -target_alt_);
            if ((now() - last_hover_log_).seconds() >= 0.5) {
                last_hover_log_ = now();
                RCLCPP_INFO(get_logger(),
                    "[TARGET_HOVER] t=%.1f x=%.2f y=%.2f z=%.2f",
                    (now() - state_start_).seconds(),
                    x_, y_, z_);
            }
            if ((now() - state_start_).seconds() > 10.0) {
                RCLCPP_INFO(get_logger(), "[STATE] LAND");
                state_ = State::LAND;
            }
            break;

        case State::LAND:
            sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            state_ = State::FINISHED;
            break;

        case State::FINISHED:
            break;
        }
    }

    float dist2D() {
        return std::hypot(target_x_ - x_, target_y_ - y_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
