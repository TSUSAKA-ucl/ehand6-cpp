// ehand_can.hpp - HitBot eHand-6 CAN FD (SocketCAN) minimal driver
// Written from the "External Communication Protocol" section of the
// EHand-6 user manual. Header-only, C++17, Linux only, no external deps.
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <system_error>

namespace ehand {

// ---------------------------------------------------------------- constants
constexpr uint32_t kRightHandId = 0x11;   // spec: right-hand ID 0x11
constexpr uint32_t kLeftHandId  = 0x12;   // spec: left-hand ID 0x12
constexpr size_t   kPayloadLen  = 32;     // effective application frame size
constexpr size_t   kNumMotors   = 6;
constexpr uint8_t  kAllMotors   = 0x3F;   // 6-bit mask, bit0 = motor 1

// Motor order = group order in the frame (motor No.1..6)
enum Motor : uint8_t {
    ThumbH = 0,  // "horizontal thumb" (lateral rotation)
    ThumbV,      // "vertical thumb"   (flexion)
    Index,
    Middle,
    Ring,
    Little       // "tail finger"
};
constexpr uint8_t motorBit(Motor m) { return static_cast<uint8_t>(1u << m); }

// Control word, lower 4 bits
enum class Mode : uint8_t {
    Disable   = 0,
    Position  = 1,  // move to position using pos/speed/torque
    Extend    = 2,  // fully extend finger (uses speed/torque)
    Grip      = 3,  // close finger        (uses speed/torque)
    Reset     = 4,  // zero-reset
    SavePoint = 5,  // save given params as preset position (slot in upper nibble)
    ExecPoint = 6   // execute preset position               (slot in upper nibble)
};

// Lower 2 bits of byte 1
enum class MsgType : uint8_t {
    FormatError   = 0,
    ActiveUpload  = 1,  // (command frame: Write=1)
    ReadResponse  = 2,
    WriteResponse = 3
};

// state nibble (4.2)
enum class JointStateCode : uint8_t {
    Init = 0, Standby = 1, Calibrating = 2, PositionMode = 3, Reserved4 = 4,
    Aging = 5, Fault = 6, WaitingResponse = 7
};

inline const char* stateName(uint8_t s) {
    switch (s) {
        case 0: return "init";
        case 1: return "standby";
        case 2: return "calibrating";
        case 3: return "position mode";
        case 5: return "aging";
        case 6: return "fault";
        case 7: return "waiting response";
        default: return "reserved";
    }
}
// fault code nibble (4.2)
inline const char* faultName(uint8_t f) {
    switch (f) {
        case 0: return "none";
        case 1: return "overcurrent";
        case 2: return "overvoltage";
        case 3: return "undervoltage";
        case 4: return "overheat";
        case 5: return "stall";
        case 6: return "comm timeout";
        case 7: return "hardware";
        default: return "reserved";
    }
}

// ---------------------------------------------------------------- helpers
// percent (0..100) <-> raw (0..255). Spec: 0 -> 0 %, 1..255 -> 1..100 %.
inline uint8_t percentToRaw(double pct) {
    pct = std::clamp(pct, 0.0, 100.0);
    return static_cast<uint8_t>(std::lround(pct * 255.0 / 100.0));
}
inline double rawToPercent(uint8_t raw) { return raw * 100.0 / 255.0; }

// ---------------------------------------------------------------- data types
struct JointCmd {
    uint8_t position = 0x80;  // 0x80 = half stroke
    uint8_t speed    = 0x80;  // 0x80 = half speed
    uint8_t torque   = 0xCC;  // 0xCC = 80 % torque

    static JointCmd fromPercent(double pos_pct, double speed_pct, double torque_pct) {
        return {percentToRaw(pos_pct), percentToRaw(speed_pct), percentToRaw(torque_pct)};
    }
};
using JointCmds = std::array<JointCmd, kNumMotors>;
using Payload   = std::array<uint8_t, kPayloadLen>;

struct JointState {
    uint8_t state    = 0;   // see stateName()
    uint8_t fault    = 0;   // see faultName()
    uint8_t position = 0;   // raw 0..255
    uint8_t speed    = 0;   // raw 0..255
    double  position_pct() const { return rawToPercent(position); }
    double  speed_pct()    const { return rawToPercent(speed); }
};

struct HandState {
    MsgType msg_type   = MsgType::FormatError;
    uint8_t motor_mask = 0;      // 6-bit ID field of byte 1
    uint8_t sys_state  = 0;      // byte 2 low nibble
    uint8_t sys_fault  = 0;      // byte 2 high nibble
    std::array<JointState, kNumMotors> joint{};
    Payload raw{};               // untouched 32 bytes for debugging
};

// ---------------------------------------------------------------- packet codec
// Byte 1 = (motor_mask << 2) | op   (op: 0 = read, 1 = write)
inline uint8_t makeHeader(uint8_t motor_mask, uint8_t op) {
    return static_cast<uint8_t>(((motor_mask & 0x3F) << 2) | (op & 0x03));
}

// Write command. Bytes 3..32 = 6 groups x [position, speed, torque, rsv, rsv]
inline Payload buildWriteCommand(uint8_t motor_mask, Mode mode, const JointCmds& j,
                                 uint8_t slot = 0) {
    Payload p{};  // zero-filled (reserved bytes = 0)
    p[0] = makeHeader(motor_mask, 1);
    p[1] = static_cast<uint8_t>((static_cast<uint8_t>(mode) & 0x0F) | ((slot & 0x0F) << 4));
    for (size_t i = 0; i < kNumMotors; ++i) {
        const size_t b = 2 + 5 * i;
        p[b + 0] = j[i].position;
        p[b + 1] = j[i].speed;
        p[b + 2] = j[i].torque;
    }
    return p;
}

// Read (state request). Only byte 1 matters; the rest is zero padding.
inline Payload buildReadCommand(uint8_t motor_mask = kAllMotors) {
    Payload p{};
    p[0] = makeHeader(motor_mask, 0);  // all motors -> 0xFC
    return p;
}

// Feedback frame. Per the byte table (4.1):
// bytes 3..32 = 6 groups x [state(lo4)|fault(hi4), position, speed, rsv, rsv]
inline bool parseState(const uint8_t* d, size_t len, HandState& out) {
    if (len < kPayloadLen) return false;
    std::memcpy(out.raw.data(), d, kPayloadLen);
    out.msg_type   = static_cast<MsgType>(d[0] & 0x03);
    out.motor_mask = d[0] >> 2;
    out.sys_state  = d[1] & 0x0F;
    out.sys_fault  = d[1] >> 4;
    for (size_t i = 0; i < kNumMotors; ++i) {
        const size_t b = 2 + 5 * i;
        out.joint[i].state    = d[b] & 0x0F;
        out.joint[i].fault    = d[b] >> 4;
        out.joint[i].position = d[b + 1];
        out.joint[i].speed    = d[b + 2];
    }
    return true;
}

// ---------------------------------------------------------------- driver
class EHandCan {

public:
    // rx_id: CAN ID of feedback frames. 0xFFFFFFFF = accept any ID.
    // (The manual does not state the feedback ID explicitly; default = same as tx.)
    static constexpr uint32_t kAnyId = 0xFFFFFFFFu;

    explicit EHandCan(const std::string& ifname = "can0",
                      uint32_t tx_id = kRightHandId,
                      uint32_t rx_id = 0, bool brs = true)
        : tx_id_(tx_id), rx_id_(rx_id ? rx_id : tx_id), brs_(brs) {
        fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) throwErrno("socket");

        int enable = 1;
        if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof enable) < 0) {
            int e = errno; ::close(fd_); throw std::system_error(e, std::generic_category(),
                                       "CAN_RAW_FD_FRAMES (kernel/driver without CAN FD?)");
        }
        sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = static_cast<int>(::if_nametoindex(ifname.c_str()));
        if (addr.can_ifindex == 0) {
            ::close(fd_);
            throw std::runtime_error("interface not found: " + ifname);
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
            int e = errno; ::close(fd_);
            throw std::system_error(e, std::generic_category(), "bind");
        }
    }
    ~EHandCan() { if (fd_ >= 0) ::close(fd_); }
    EHandCan(const EHandCan&) = delete;
    EHandCan& operator=(const EHandCan&) = delete;

    // ------------------------------------------------ send side
    void sendPayload(const Payload& p) {
        canfd_frame f{};
        f.can_id = tx_id_;                       // standard 11-bit ID
        f.len    = static_cast<__u8>(kPayloadLen);  // 32 bytes (valid CAN FD length)
        f.flags  = (brs_ ? CANFD_BRS : 0);
#ifdef CANFD_FDF
        f.flags |= CANFD_FDF;
#endif
        std::memcpy(f.data, p.data(), kPayloadLen);
        ssize_t n = ::write(fd_, &f, CANFD_MTU);
        if (n != static_cast<ssize_t>(CANFD_MTU)) throwErrno("write");
    }

    // Generic write command
    void sendCommand(uint8_t motor_mask, Mode mode, const JointCmds& j, uint8_t slot = 0) {
        sendPayload(buildWriteCommand(motor_mask, mode, j, slot));
    }

    // Position mode: move selected motors to position with speed/torque
    void movePosition(const JointCmds& j, uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::Position, j);
    }
    // Extend (open) / Grip (close): position field is ignored by the hand
    void extend(const JointCmds& j, uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::Extend, j);
    }
    void grip(const JointCmds& j, uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::Grip, j);
    }
    void zeroReset(uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::Reset, JointCmds{});
    }
    // Preset positions: slot 1.. (spec examples: 0x15 save #1, 0x16 execute #1)
    void savePoint(const JointCmds& j, uint8_t slot = 1, uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::SavePoint, j, slot);
    }
    void executePoint(uint8_t slot = 1, uint8_t motor_mask = kAllMotors) {
        sendCommand(motor_mask, Mode::ExecPoint, JointCmds{}, slot);
    }
    // Emergency stop (7.2): broadcast, control word = 0
    void emergencyStop() { sendCommand(kAllMotors, Mode::Disable, JointCmds{}); }

    // ------------------------------------------------ receive side
    // Discard everything currently queued in the socket.
    void flushRx() {
        canfd_frame f;
        while (::recv(fd_, &f, sizeof f, MSG_DONTWAIT) > 0) {}
    }

    // Send a read request and wait for the Read response (type 2).
    // Returns false on timeout.
    bool queryState(HandState& out, int timeout_ms = 100, uint8_t motor_mask = kAllMotors) {
        flushRx();
        sendPayload(buildReadCommand(motor_mask));
        return receiveState(out, timeout_ms, MsgType::ReadResponse);
    }

    // Wait for the next state-carrying frame of the given type
    // (e.g. MsgType::ActiveUpload for unsolicited error/status uploads).
    bool receiveState(HandState& out, int timeout_ms, MsgType want = MsgType::ReadResponse) {
        using clk = std::chrono::steady_clock;
        const auto deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
        for (;;) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  deadline - clk::now()).count();
            if (left < 0) return false;

            canfd_frame f;
            if (!recvFrame(f, static_cast<int>(left))) return false;
            if (f.can_id & CAN_ERR_FLAG) continue;                       // bus error frame
            if (rx_id_ != kAnyId && (f.can_id & CAN_EFF_MASK) != rx_id_) continue;

            HandState tmp;
            if (!parseState(f.data, f.len, tmp)) continue;
            if (tmp.msg_type == want) { out = tmp; return true; }
            // write responses / other types are skipped
        }
    }

private:
    // Returns true if a frame was read; false on timeout.
    bool recvFrame(canfd_frame& f, int timeout_ms) {
        pollfd pfd{fd_, POLLIN, 0};
        int r = ::poll(&pfd, 1, timeout_ms < 0 ? 0 : timeout_ms);
        if (r == 0) return false;
        if (r < 0) { if (errno == EINTR) return false; throwErrno("poll"); }
        std::memset(&f, 0, sizeof f);
        ssize_t n = ::read(fd_, &f, sizeof f);
        if (n < 0) { if (errno == EAGAIN || errno == EINTR) return false; throwErrno("read"); }
        // classic CAN (16 B) and CAN FD (72 B) share can_id/len/data offsets
        return n == static_cast<ssize_t>(CAN_MTU) || n == static_cast<ssize_t>(CANFD_MTU);
    }
    [[noreturn]] static void throwErrno(const char* what) {
        throw std::system_error(errno, std::generic_category(), what);
    }

    int      fd_ = -1;
    uint32_t tx_id_, rx_id_;
    bool     brs_;
};

}  // namespace ehand
