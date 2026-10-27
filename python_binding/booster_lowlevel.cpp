// booster_lowlevel: a small Python binding over this SDK's low-level channels and the
// locomotion RPC's mode switch, for one Booster robot (K1, T1 or T2), joints addressed by
// their serial index (the robot's joint order, as `motor_state_serial` lists them).
//
//   rt/low_state                the IMU (rpy, gyro, acc) and the serial joint states
//   rt/remote_controller_state  the Booster remote's buttons and sticks
//   rt/joint_ctrl               one SERIAL LowCmd per `write_low_command` call
//   rt/LocoApiTopic             ChangeMode / GetMode (joint_ctrl acts only in Custom mode)
//
// The subscription handlers keep the latest message of each topic with its arrival time,
// so the reads never block. Commands are written when called, not re-sent by a thread:
// the robot's low-level controller keeps the last command it received.
//
// `FakeRobot` is the robot's side of the same topics and RPC, for desk tests without a
// robot (a simulator plays the robot). Both share the process's one DDS participant (the
// SDK's `ChannelFactory`), so they also talk within one process. That participant keeps the
// network interface and domain of the first object a process creates.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_api.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/error.hpp>
#include <booster/robot/rpc/rpc_server.hpp>

namespace py = pybind11;
namespace msg = booster_interface::msg;
using booster::robot::ChannelFactory;
using booster::robot::ChannelPublisher;
using booster::robot::ChannelSubscriber;
using booster::robot::RobotMode;
using booster::robot::b1::LocoApiId;
using Clock = std::chrono::steady_clock;

static const std::string kTopicRemote = "rt/remote_controller_state";

struct ImuState {
  std::array<float, 3> rpy{};
  std::array<float, 3> gyro{};
  std::array<float, 3> acc{};
};

struct MotorState {
  std::vector<float> q, dq, tau_est;
  std::vector<int> temperature;
  std::vector<uint32_t> lost;
};

struct LowState {
  ImuState imu;
  MotorState motor;  // As many joints as the message lists (empty before the first).
  double age = std::numeric_limits<double>::infinity();  // Seconds since it arrived.
  uint64_t count = 0;  // Messages received so far.
};

struct MotorCommand {
  std::vector<float> q_target, dq_target, kp, kd, tau_ff;
  explicit MotorCommand(int n) : q_target(n), dq_target(n), kp(n), kd(n), tau_ff(n) {}
};

struct RemoteController {
  bool a = false, b = false, x = false, y = false, lb = false, rb = false, lt = false,
       rt = false, ls = false, rs = false, back = false, start = false, hat_u = false,
       hat_d = false, hat_l = false, hat_r = false;
  float lx = 0, ly = 0, rx = 0, ry = 0;
  double age = std::numeric_limits<double>::infinity();
};

static double seconds_since(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now() - t).count();
}

class BoosterInterface {
 public:
  BoosterInterface(const std::string &network_interface, int num_motors, int domain_id)
      : n_(num_motors),
        state_sub_(booster::robot::b1::kTopicLowState),
        remote_sub_(kTopicRemote),
        cmd_pub_(booster::robot::b1::kTopicJointCtrl) {
    ChannelFactory::Instance()->Init(domain_id, network_interface);
    state_sub_.InitChannel([this](const void *m) { OnLowState(*static_cast<const msg::LowState *>(m)); });
    remote_sub_.InitChannel(
        [this](const void *m) { OnRemote(*static_cast<const msg::RemoteControllerState *>(m)); });
    cmd_pub_.InitChannel();
    loco_.Init();
    cmd_.cmd_type(msg::CmdType::SERIAL);
    cmd_.motor_cmd().resize(n_);
  }

  ~BoosterInterface() {
    state_sub_.CloseChannel();
    remote_sub_.CloseChannel();
    cmd_pub_.CloseChannel();
  }

  int num_motors() const { return n_; }

  LowState read_low_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    LowState out = state_;
    if (state_.count > 0) out.age = seconds_since(state_time_);
    return out;
  }

  // The latest buttons, each also true if it was down in any message since the last read:
  // the remote sends events, so a short press can be gone from the latest message.
  RemoteController read_remote_controller() {
    std::lock_guard<std::mutex> lock(mutex_);
    RemoteController out = remote_;
    for (size_t i = 0; i < kButtons; ++i) *Button(out, i) = *Button(out, i) || *Button(pressed_, i);
    pressed_ = RemoteController{};
    if (remote_seen_) out.age = seconds_since(remote_time_);
    return out;
  }

  MotorCommand create_zero_command() const { return MotorCommand(n_); }

  // One SERIAL command, now. `weight` stays 0 and `mode` 0, as Booster's own deploy sends.
  void write_low_command(const MotorCommand &c) {
    for (const auto *v : {&c.q_target, &c.dq_target, &c.kp, &c.kd, &c.tau_ff}) {
      if (static_cast<int>(v->size()) != n_)
        throw std::invalid_argument("command lists " + std::to_string(v->size()) +
                                    " joints, expected " + std::to_string(n_));
    }
    auto &motors = cmd_.motor_cmd();
    for (int i = 0; i < n_; ++i) {
      motors[i].q(c.q_target[i]);
      motors[i].dq(c.dq_target[i]);
      motors[i].kp(c.kp[i]);
      motors[i].kd(c.kd[i]);
      motors[i].tau(c.tau_ff[i]);
    }
    cmd_pub_.Write(&cmd_);
  }

  // The RPC's status (0: success; 100: no answer within the SDK's timeout). Modes are the
  // SDK's `RobotMode` values as ints (`RobotMode.CUSTOM` passes as one).
  int change_mode(int mode) { return loco_.ChangeMode(static_cast<RobotMode>(mode)); }

  // The robot's mode, or -1 if the RPC failed (no answer: after 5 s).
  int get_mode() {
    booster::robot::b1::GetModeResponse resp;
    return loco_.GetMode(resp) == 0 ? static_cast<int>(resp.mode_) : -1;
  }

  size_t low_state_publishers() const { return state_sub_.GetMatchedPublicationsCount(); }
  size_t command_subscribers() const { return cmd_pub_.GetMatchedSubscriptionsCount(); }

 private:
  static constexpr size_t kButtons = 16;

  // The i-th button of `r`, in the struct's order (for the latch).
  static bool *Button(RemoteController &r, size_t i) {
    bool *buttons[kButtons] = {&r.a,  &r.b,  &r.x,    &r.y,     &r.lb,    &r.rb,    &r.lt,    &r.rt,
                               &r.ls, &r.rs, &r.back, &r.start, &r.hat_u, &r.hat_d, &r.hat_l, &r.hat_r};
    return buttons[i];
  }

  void OnLowState(const msg::LowState &m) {
    LowState s;
    const auto &imu = m.imu_state();
    s.imu.rpy = imu.rpy();
    s.imu.gyro = imu.gyro();
    s.imu.acc = imu.acc();
    for (const auto &j : m.motor_state_serial()) {
      s.motor.q.push_back(j.q());
      s.motor.dq.push_back(j.dq());
      s.motor.tau_est.push_back(j.tau_est());
      s.motor.temperature.push_back(j.temperature());
      s.motor.lost.push_back(j.lost());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    s.count = state_.count + 1;
    state_ = std::move(s);
    state_time_ = Clock::now();
  }

  void OnRemote(const msg::RemoteControllerState &m) {
    RemoteController r;
    r.a = m.a(), r.b = m.b(), r.x = m.x(), r.y = m.y(), r.lb = m.lb(), r.rb = m.rb();
    r.lt = m.lt(), r.rt = m.rt(), r.ls = m.ls(), r.rs = m.rs(), r.back = m.back();
    r.start = m.start(), r.hat_u = m.hat_u(), r.hat_d = m.hat_d(), r.hat_l = m.hat_l();
    r.hat_r = m.hat_r(), r.lx = m.lx(), r.ly = m.ly(), r.rx = m.rx(), r.ry = m.ry();
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < kButtons; ++i) *Button(pressed_, i) = *Button(pressed_, i) || *Button(r, i);
    remote_ = r;
    remote_time_ = Clock::now();
    remote_seen_ = true;
  }

  const int n_;
  std::mutex mutex_;
  LowState state_;
  Clock::time_point state_time_;
  RemoteController remote_, pressed_;
  Clock::time_point remote_time_;
  bool remote_seen_ = false;
  ChannelSubscriber<msg::LowState> state_sub_;
  ChannelSubscriber<msg::RemoteControllerState> remote_sub_;
  ChannelPublisher<msg::LowCmd> cmd_pub_;
  booster::robot::b1::B1LocoClient loco_;
  msg::LowCmd cmd_;
};

// The robot's side: publishes the state and the remote, keeps the latest joint command and
// answers ChangeMode and GetMode from the mode it keeps (other RPCs: bad request).
class FakeRobot : public booster::robot::RpcServer {
 public:
  FakeRobot(const std::string &network_interface, int num_motors, int domain_id, int mode)
      : n_(num_motors),
        mode_(mode),
        state_pub_(booster::robot::b1::kTopicLowState),
        remote_pub_(kTopicRemote),
        cmd_sub_(booster::robot::b1::kTopicJointCtrl) {
    ChannelFactory::Instance()->Init(domain_id, network_interface);
    state_pub_.InitChannel();
    remote_pub_.InitChannel();
    cmd_sub_.InitChannel([this](const void *m) { OnCommand(*static_cast<const msg::LowCmd *>(m)); });
    Init(booster::robot::b1::kLocoApiChannelName);
  }

  ~FakeRobot() override {
    Stop();
    cmd_sub_.CloseChannel();
    state_pub_.CloseChannel();
    remote_pub_.CloseChannel();
  }

  void publish_low_state(const std::array<float, 3> &rpy, const std::array<float, 3> &gyro,
                         const std::array<float, 3> &acc, const std::vector<float> &q,
                         const std::vector<float> &dq) {
    if (static_cast<int>(q.size()) != n_ || static_cast<int>(dq.size()) != n_)
      throw std::invalid_argument("state lists " + std::to_string(q.size()) + " joints, expected " +
                                  std::to_string(n_));
    msg::LowState s;
    s.imu_state().rpy(rpy);
    s.imu_state().gyro(gyro);
    s.imu_state().acc(acc);
    s.motor_state_serial().resize(n_);
    for (int i = 0; i < n_; ++i) {
      s.motor_state_serial()[i].q(q[i]);
      s.motor_state_serial()[i].dq(dq[i]);
    }
    state_pub_.Write(&s);
  }

  // The remote with the named buttons (`RemoteController`'s field names) down.
  void publish_remote(const std::vector<std::string> &held) {
    static const std::map<std::string, std::function<void(msg::RemoteControllerState &)>> press = {
        {"a", [](auto &r) { r.a(true); }},         {"b", [](auto &r) { r.b(true); }},
        {"x", [](auto &r) { r.x(true); }},         {"y", [](auto &r) { r.y(true); }},
        {"lb", [](auto &r) { r.lb(true); }},       {"rb", [](auto &r) { r.rb(true); }},
        {"lt", [](auto &r) { r.lt(true); }},       {"rt", [](auto &r) { r.rt(true); }},
        {"ls", [](auto &r) { r.ls(true); }},       {"rs", [](auto &r) { r.rs(true); }},
        {"back", [](auto &r) { r.back(true); }},   {"start", [](auto &r) { r.start(true); }},
        {"hat_u", [](auto &r) { r.hat_u(true); }}, {"hat_d", [](auto &r) { r.hat_d(true); }},
        {"hat_l", [](auto &r) { r.hat_l(true); }}, {"hat_r", [](auto &r) { r.hat_r(true); }},
    };
    msg::RemoteControllerState r;
    for (const auto &name : held) {
      auto it = press.find(name);
      if (it == press.end()) throw std::invalid_argument("no button " + name);
      it->second(r);
    }
    remote_pub_.Write(&r);
  }

  // The latest joint command (None before the first), and how many arrived so far.
  std::pair<std::optional<MotorCommand>, uint64_t> read_command() {
    std::lock_guard<std::mutex> lock(mutex_);
    return {cmd_, cmd_count_};
  }

  int mode() {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
  }

  void set_mode(int mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
  }

  size_t command_publishers() const { return cmd_sub_.GetMatchedPublicationsCount(); }

 protected:
  booster::robot::Response HandleRequest(const booster::robot::Request &req) override {
    using booster::robot::Response;
    using booster::robot::ResponseHeader;
    const auto api = static_cast<LocoApiId>(req.GetHeader().GetApiId());
    std::lock_guard<std::mutex> lock(mutex_);
    if (api == LocoApiId::kChangeMode) {
      mode_ = nlohmann::json::parse(req.GetBody())["mode"].get<int>();
      return Response::OkResponse();
    }
    if (api == LocoApiId::kGetMode) {
      nlohmann::json body;
      body["mode"] = mode_;
      return Response(ResponseHeader(booster::robot::kRpcStatusCodeSuccess), body.dump());
    }
    return Response(ResponseHeader(booster::robot::kRpcStatusCodeBadRequest), "");
  }

 private:
  void OnCommand(const msg::LowCmd &m) {
    if (m.cmd_type() != msg::CmdType::SERIAL || static_cast<int>(m.motor_cmd().size()) != n_) return;
    MotorCommand c(n_);
    for (int i = 0; i < n_; ++i) {
      const auto &j = m.motor_cmd()[i];
      c.q_target[i] = j.q(), c.dq_target[i] = j.dq(), c.kp[i] = j.kp(), c.kd[i] = j.kd();
      c.tau_ff[i] = j.tau();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = std::move(c);
    ++cmd_count_;
  }

  const int n_;
  std::mutex mutex_;
  int mode_;
  std::optional<MotorCommand> cmd_;
  uint64_t cmd_count_ = 0;
  ChannelPublisher<msg::LowState> state_pub_;
  ChannelPublisher<msg::RemoteControllerState> remote_pub_;
  ChannelSubscriber<msg::LowCmd> cmd_sub_;
};

PYBIND11_MODULE(booster_lowlevel, m) {
  m.doc() = "Low-level I/O and the mode switch of a Booster robot over the Booster Robotics SDK";
  m.attr("T2_NUM_MOTORS") = 31;

  py::enum_<RobotMode>(m, "RobotMode")
      .value("DAMPING", RobotMode::kDamping)
      .value("PREPARE", RobotMode::kPrepare)
      .value("WALKING", RobotMode::kWalking)
      .value("CUSTOM", RobotMode::kCustom)
      .export_values();

  py::class_<ImuState>(m, "ImuState")
      .def_readonly("rpy", &ImuState::rpy)
      .def_readonly("gyro", &ImuState::gyro)
      .def_readonly("acc", &ImuState::acc);

  py::class_<MotorState>(m, "MotorState")
      .def_readonly("q", &MotorState::q)
      .def_readonly("dq", &MotorState::dq)
      .def_readonly("tau_est", &MotorState::tau_est)
      .def_readonly("temperature", &MotorState::temperature)
      .def_readonly("lost", &MotorState::lost);

  py::class_<LowState>(m, "LowState")
      .def_readonly("imu", &LowState::imu)
      .def_readonly("motor", &LowState::motor)
      .def_readonly("age", &LowState::age)
      .def_readonly("count", &LowState::count);

  py::class_<MotorCommand>(m, "MotorCommand")
      .def(py::init<int>())
      .def_readwrite("q_target", &MotorCommand::q_target)
      .def_readwrite("dq_target", &MotorCommand::dq_target)
      .def_readwrite("kp", &MotorCommand::kp)
      .def_readwrite("kd", &MotorCommand::kd)
      .def_readwrite("tau_ff", &MotorCommand::tau_ff);

  py::class_<RemoteController>(m, "RemoteController")
      .def_readonly("a", &RemoteController::a)
      .def_readonly("b", &RemoteController::b)
      .def_readonly("x", &RemoteController::x)
      .def_readonly("y", &RemoteController::y)
      .def_readonly("lb", &RemoteController::lb)
      .def_readonly("rb", &RemoteController::rb)
      .def_readonly("lt", &RemoteController::lt)
      .def_readonly("rt", &RemoteController::rt)
      .def_readonly("ls", &RemoteController::ls)
      .def_readonly("rs", &RemoteController::rs)
      .def_readonly("back", &RemoteController::back)
      .def_readonly("start", &RemoteController::start)
      .def_readonly("hat_u", &RemoteController::hat_u)
      .def_readonly("hat_d", &RemoteController::hat_d)
      .def_readonly("hat_l", &RemoteController::hat_l)
      .def_readonly("hat_r", &RemoteController::hat_r)
      .def_readonly("lx", &RemoteController::lx)
      .def_readonly("ly", &RemoteController::ly)
      .def_readonly("rx", &RemoteController::rx)
      .def_readonly("ry", &RemoteController::ry)
      .def_readonly("age", &RemoteController::age);

  py::class_<BoosterInterface>(m, "BoosterInterface")
      .def(py::init<const std::string &, int, int>(), py::arg("network_interface"),
           py::arg("num_motors"), py::arg("domain_id") = 0)
      .def_static(
          "create_t2",
          [](const std::string &net, int domain_id) {
            return std::make_unique<BoosterInterface>(net, 31, domain_id);
          },
          py::arg("network_interface"), py::arg("domain_id") = 0)
      .def("num_motors", &BoosterInterface::num_motors)
      .def("read_low_state", &BoosterInterface::read_low_state)
      .def("read_remote_controller", &BoosterInterface::read_remote_controller)
      .def("create_zero_command", &BoosterInterface::create_zero_command)
      .def("write_low_command", &BoosterInterface::write_low_command)
      .def("change_mode", &BoosterInterface::change_mode, py::arg("mode"),
           py::call_guard<py::gil_scoped_release>())
      .def("get_mode", &BoosterInterface::get_mode, py::call_guard<py::gil_scoped_release>())
      .def("low_state_publishers", &BoosterInterface::low_state_publishers)
      .def("command_subscribers", &BoosterInterface::command_subscribers);

  py::class_<FakeRobot>(m, "FakeRobot")
      .def(py::init<const std::string &, int, int, int>(), py::arg("network_interface"),
           py::arg("num_motors"), py::arg("domain_id") = 0,
           py::arg("mode") = static_cast<int>(RobotMode::kPrepare))
      .def("publish_low_state", &FakeRobot::publish_low_state, py::arg("rpy"), py::arg("gyro"),
           py::arg("acc"), py::arg("q"), py::arg("dq"))
      .def("publish_remote", &FakeRobot::publish_remote, py::arg("held"))
      .def("read_command", &FakeRobot::read_command)
      .def("mode", &FakeRobot::mode)
      .def("set_mode", &FakeRobot::set_mode, py::arg("mode"))
      .def("command_publishers", &FakeRobot::command_publishers);
}
