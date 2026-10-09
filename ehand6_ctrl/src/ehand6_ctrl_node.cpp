// ehand6_ctrl_node.cpp
//
//  sub  joint_command  (sensor_msgs/JointState)   normalized 0..1 position/velocity/effort
//  pub  hand_state     (ehand6_msgs/HandState)    published on every state query
//  srv  reset          (std_srvs/Trigger)         zeroing; returns when all joints are STANDBY
//  srv  emergency_stop (std_srvs/Trigger)         broadcast stop (control word = 0)
//
// All CAN access is serialized with can_mtx_ (queryState() flushes the RX queue,
// so concurrent access would make callers steal each other's frames).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ehand6_ctrl/ehand_can.hpp"
#include "ehand6_msgs/msg/hand_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using Trigger      = std_srvs::srv::Trigger;
using HandStateMsg = ehand6_msgs::msg::HandState;
using SteadyClock  = std::chrono::steady_clock;

namespace {
constexpr uint8_t kStateStandby = 1;
constexpr uint8_t kStateFault   = 6;

uint8_t normToRaw(double v) {
    v = std::clamp(v, 0.0, 1.0);
    return static_cast<uint8_t>(std::lround(v * 255.0));
}
float rawToNorm(uint8_t r) { return static_cast<float>(r) / 255.0f; }
}  // namespace

class EHandNode : public rclcpp::Node {
public:
    EHandNode() : Node("ehand6_ctrl") {
        // ---- parameters
        const auto can_if = declare_parameter<std::string>("can_interface", "can0");
        const auto side   = declare_parameter<std::string>("side", "right");
        const auto rx_id  = declare_parameter<int>("rx_id", 0);
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", {"thumb_h", "thumb_v", "index", "middle", "ring", "little"});
        default_velocity_ = declare_parameter<double>("default_velocity", 0.5);
        default_effort_   = declare_parameter<double>("default_effort", 0.8);
        query_timeout_ms_ = declare_parameter<int>("query_timeout_ms", 30);
        const double idle_period  = declare_parameter<double>("idle_poll_period_s", 1.0);
        const double move_period  = declare_parameter<double>("move_poll_period_s", 0.05);
        settle_tol_raw_ = static_cast<int>(
            std::lround(declare_parameter<double>("move_settle_tolerance", 0.02) * 255.0));
        stall_time_s_       = declare_parameter<double>("move_stall_time_s", 0.5);
        move_timeout_s_     = declare_parameter<double>("move_timeout_s", 5.0);
        reset_timeout_s_    = declare_parameter<double>("reset_timeout_s", 15.0);
        reset_poll_s_       = declare_parameter<double>("reset_poll_period_s", 0.05);
        reset_grace_s_      = declare_parameter<double>("reset_start_grace_s", 1.0);

        if (joint_names_.size() != ehand::kNumMotors)
            throw std::runtime_error("joint_names must have exactly 6 entries");
        if (side != "right" && side != "left")
            throw std::runtime_error("side must be 'right' or 'left'");

        // ---- CAN
        const uint32_t tx_id = (side == "left") ? ehand::kLeftHandId : ehand::kRightHandId;
        const uint32_t rx = rx_id < 0 ? ehand::EHandCan::kAnyId : static_cast<uint32_t>(rx_id);
        hand_ = std::make_unique<ehand::EHandCan>(can_if, tx_id, rx);
        RCLCPP_INFO(get_logger(), "eHand-6 (%s hand, tx id 0x%02X) on %s",
                    side.c_str(), tx_id, can_if.c_str());

        // ---- callback groups (so a long reset never blocks commands / timers)
        grp_sub_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        grp_timer_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        grp_reset_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        grp_estop_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // ---- interfaces
        pub_ = create_publisher<HandStateMsg>("hand_state", 10);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = grp_sub_;
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_command", 10, std::bind(&EHandNode::onJointCommand, this, _1), sub_opts);

        reset_srv_ = create_service<Trigger>(
            "reset", std::bind(&EHandNode::onReset, this, _1, _2),
            rclcpp::ServicesQoS(), grp_reset_);
        estop_srv_ = create_service<Trigger>(
            "emergency_stop", std::bind(&EHandNode::onEmergencyStop, this, _1, _2),
            rclcpp::ServicesQoS(), grp_estop_);

        move_timer_ = create_wall_timer(std::chrono::duration<double>(move_period),
                                        std::bind(&EHandNode::onMoveTimer, this), grp_timer_);
        if (idle_period > 0.0)
            idle_timer_ = create_wall_timer(std::chrono::duration<double>(idle_period),
                                            std::bind(&EHandNode::onIdleTimer, this), grp_timer_);
    }

private:
    // ================================================================ state query + publish
    // Every state query in this node goes through here, so every query is published.
    bool pollState(const char* source, ehand::HandState* out = nullptr) {
        ehand::HandState st;
        bool ok = false;
        try {
            std::lock_guard<std::mutex> lk(can_mtx_);
            ok = hand_->queryState(st, query_timeout_ms_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "CAN error: %s", e.what());
            return false;
        }
        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "state query timed out");
            return false;
        }
        pub_->publish(toMsg(st, source));
        if (out) *out = st;
        return true;
    }

    HandStateMsg toMsg(const ehand::HandState& s, const char* source) {
        HandStateMsg m;
        m.header.stamp = now();
        m.source       = source;
        m.msg_type     = static_cast<uint8_t>(s.msg_type);
        m.motor_mask   = s.motor_mask;
        m.system_state = s.sys_state;
        m.system_fault = s.sys_fault;
        bool all = true;
        for (size_t i = 0; i < ehand::kNumMotors; ++i) {
            auto& j    = m.joints[i];
            j.name     = joint_names_[i];
            j.state    = s.joint[i].state;
            j.fault    = s.joint[i].fault;
            j.position = rawToNorm(s.joint[i].position);
            j.velocity = rawToNorm(s.joint[i].speed);
            all &= (s.joint[i].state == kStateStandby);
        }
        m.all_standby = all;
        std::copy(s.raw.begin(), s.raw.end(), m.raw.begin());
        return m;
    }

    // ================================================================ move
    void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (resetting_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "joint_command ignored while resetting");
            return;
        }
        const size_t n = msg->position.size();
        if (n == 0) return;

        // name -> joint index (empty names => positional, must be 6 entries)
        std::vector<int> idx(n, -1);
        if (msg->name.empty()) {
            if (n != ehand::kNumMotors) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "no names given: position must have 6 entries");
                return;
            }
            for (size_t i = 0; i < n; ++i) idx[i] = static_cast<int>(i);
        } else {
            if (msg->name.size() != n) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "name/position size mismatch");
                return;
            }
            for (size_t i = 0; i < n; ++i) {
                auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
                if (it != joint_names_.end())
                    idx[i] = static_cast<int>(it - joint_names_.begin());
                else
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "unknown joint name '%s'", msg->name[i].c_str());
            }
        }
        const bool vel_ok = msg->velocity.size() == n;
        const bool eff_ok = msg->effort.size() == n;

        ehand::JointCmds cmd{};
        std::array<uint8_t, ehand::kNumMotors> target{};
        uint8_t mask = 0;
        for (size_t i = 0; i < n; ++i) {
            const int k = idx[i];
            if (k < 0 || !std::isfinite(msg->position[i])) continue;
            const double v = (vel_ok && std::isfinite(msg->velocity[i])) ? msg->velocity[i]
                                                                         : default_velocity_;
            const double e = (eff_ok && std::isfinite(msg->effort[i])) ? msg->effort[i]
                                                                       : default_effort_;
            cmd[k].position = normToRaw(msg->position[i]);
            cmd[k].speed    = normToRaw(v);
            cmd[k].torque   = normToRaw(e);
            target[k]       = cmd[k].position;
            mask |= ehand::motorBit(static_cast<ehand::Motor>(k));
        }
        if (!mask) return;

        try {
            std::lock_guard<std::mutex> lk(can_mtx_);
            hand_->movePosition(cmd, mask);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "send failed: %s", e.what());
            return;
        }
        startMonitor(mask, target);
    }

    // ---- "moving" monitor: publish state periodically until the move has settled
    struct Monitor {
        bool active = false;
        uint8_t mask = 0;
        std::array<uint8_t, ehand::kNumMotors> target{};
        std::array<uint8_t, ehand::kNumMotors> last_pos{};
        bool have_last = false;
        SteadyClock::time_point last_change, deadline;
    };

    void startMonitor(uint8_t mask, const std::array<uint8_t, ehand::kNumMotors>& target) {
        std::lock_guard<std::mutex> lk(mon_mtx_);
        const auto now_tp = SteadyClock::now();
        mon_.active  = true;
        mon_.mask   |= mask;
        for (size_t i = 0; i < ehand::kNumMotors; ++i)
            if (mask & (1u << i)) mon_.target[i] = target[i];
        mon_.have_last   = false;  // positions are re-baselined on the next poll
        mon_.last_change = now_tp;
        mon_.deadline    = now_tp + std::chrono::duration_cast<SteadyClock::duration>(
                                        std::chrono::duration<double>(move_timeout_s_));
    }
    void stopMonitor() {
        std::lock_guard<std::mutex> lk(mon_mtx_);
        mon_ = Monitor{};
    }

    void onMoveTimer() {
        {
            std::lock_guard<std::mutex> lk(mon_mtx_);
            if (!mon_.active) return;
        }
        if (resetting_) return;

        ehand::HandState st;
        const bool ok = pollState("move", &st);

        std::lock_guard<std::mutex> lk(mon_mtx_);
        if (!mon_.active) return;  // e.g. emergency stop in the meantime
        const auto now_tp = SteadyClock::now();
        if (!ok) {
            if (now_tp > mon_.deadline) mon_ = Monitor{};
            return;
        }
        bool reached = true, changed = !mon_.have_last, faulted = false;
        for (size_t i = 0; i < ehand::kNumMotors; ++i) {
            if (!(mon_.mask & (1u << i))) continue;
            const uint8_t pos = st.joint[i].position;
            if (std::abs(static_cast<int>(pos) - static_cast<int>(mon_.target[i])) > settle_tol_raw_)
                reached = false;
            if (mon_.have_last && pos != mon_.last_pos[i]) changed = true;
            if (st.joint[i].state == kStateFault) faulted = true;
            mon_.last_pos[i] = pos;
        }
        mon_.have_last = true;
        if (changed) mon_.last_change = now_tp;

        const bool stalled =
            std::chrono::duration<double>(now_tp - mon_.last_change).count() > stall_time_s_;
        if (reached || stalled || faulted || now_tp > mon_.deadline) mon_ = Monitor{};
    }

    void onIdleTimer() {
        if (resetting_) return;
        {
            std::lock_guard<std::mutex> lk(mon_mtx_);
            if (mon_.active) return;  // the move monitor is already publishing
        }
        pollState("poll");
    }

    // ================================================================ services
    void onReset(const std::shared_ptr<Trigger::Request>,
                 std::shared_ptr<Trigger::Response> res) {
        bool expected = false;
        if (!resetting_.compare_exchange_strong(expected, true)) {
            res->success = false;
            res->message = "reset already in progress";
            return;
        }
        struct Guard {
            std::atomic<bool>& f;
            ~Guard() { f = false; }
        } guard{resetting_};

        abort_reset_ = false;
        stopMonitor();
        res->success = false;

        try {
            std::lock_guard<std::mutex> lk(can_mtx_);
            hand_->zeroReset();
        } catch (const std::exception& e) {
            res->message = std::string("send failed: ") + e.what();
            return;
        }
        RCLCPP_INFO(get_logger(), "reset sent, waiting for all joints to reach STANDBY");

        const auto t0 = SteadyClock::now();
        const auto since = [&] { return std::chrono::duration<double>(SteadyClock::now() - t0).count(); };
        bool left_standby = false;  // saw INIT / CALIBRATING etc. after the reset command

        while (since() < reset_timeout_s_) {
            std::this_thread::sleep_for(std::chrono::duration<double>(reset_poll_s_));
            if (abort_reset_) {
                res->message = "aborted by emergency stop";
                return;
            }
            ehand::HandState st;
            if (!pollState("reset", &st)) continue;

            bool all_standby = true, fault = false;
            for (const auto& j : st.joint) {
                all_standby &= (j.state == kStateStandby);
                fault |= (j.state == kStateFault || j.fault != 0);
            }
            if (fault) {
                res->message = "fault reported during reset";
                return;
            }
            if (!all_standby) {
                left_standby = true;
            } else if (left_standby) {
                res->success = true;
                res->message = "reset done";
                return;
            } else if (since() > reset_grace_s_) {
                res->success = true;
                res->message = "all joints STANDBY; no state change observed (already zeroed?)";
                return;
            }
        }
        res->message = "timeout waiting for STANDBY";
    }

    void onEmergencyStop(const std::shared_ptr<Trigger::Request>,
                         std::shared_ptr<Trigger::Response> res) {
        abort_reset_ = true;
        stopMonitor();
        try {
            std::lock_guard<std::mutex> lk(can_mtx_);
            hand_->emergencyStop();
            res->success = true;
            res->message = "stop command sent";
        } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("send failed: ") + e.what();
        }
    }

    // ================================================================ members
    std::unique_ptr<ehand::EHandCan> hand_;
    std::mutex can_mtx_;

    std::vector<std::string> joint_names_;
    double default_velocity_{0.5}, default_effort_{0.8};
    int query_timeout_ms_{30};
    int settle_tol_raw_{5};
    double stall_time_s_{0.5}, move_timeout_s_{5.0};
    double reset_timeout_s_{15.0}, reset_poll_s_{0.05}, reset_grace_s_{1.0};

    std::atomic<bool> resetting_{false};
    std::atomic<bool> abort_reset_{false};
    Monitor mon_;
    std::mutex mon_mtx_;

    rclcpp::CallbackGroup::SharedPtr grp_sub_, grp_timer_, grp_reset_, grp_estop_;
    rclcpp::Publisher<HandStateMsg>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Service<Trigger>::SharedPtr reset_srv_, estop_srv_;
    rclcpp::TimerBase::SharedPtr move_timer_, idle_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<EHandNode>();
        rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
        exec.add_node(node);
        exec.spin();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("ehand6_ctrl"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
