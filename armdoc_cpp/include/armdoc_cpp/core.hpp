// ROS-free C++ port of the live /joint_states checks in armdoc/core.py (evaluate). Same thresholds, same estimators,
// same finding codes, so the C++ node and the Python tool agree on a stream (armdoc/test/test_cpp_equivalence.py).
//
// Covered: JOINT_STATES_NONE, JOINT_STATES_STALE, JOINT_NAN, LIMIT_POS, NEAR_LIMIT, LIMIT_VEL, EFFORT_SAT,
// NOISY_JOINTS, STREAM_LAG. Not covered (they need controllers, MoveIt or action topics): use the Python armdoc.
#pragma once
#include <algorithm>
#include <array>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace armdoc {

constexpr const char * ERROR = "ERROR";
constexpr const char * WARN = "WARN";
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

struct Finding {
  std::string level, code, msg;
};

struct Config {
  double stale = 2.0, grace = 20.0, near_limit = 0.01, vel_margin = 1.05, effort_frac = 0.95, effort_time = 2.0;
  double noise_rad = 0.002, noise_window = 1.0, noise_max_dt = 0.05, lag_max = 0.5, lag_window = 2.0;
  int noise_min_samples = 10;
};

struct Limit {
  std::optional<double> lower, upper, velocity, effort;
  bool continuous = false;
};

struct Sample {
  double t;
  std::vector<double> pos;   // indexed like State::names; samples older than a newly seen joint are shorter (NaN)
};

struct State {
  double t0 = 0.0;
  std::optional<double> joint_t;
  std::vector<std::string> names;
  std::map<std::string, std::array<double, 3>> joints;       // pos, vel, effort
  std::map<std::string, Limit> limits;
  std::optional<std::set<std::string>> arm_joints;           // none = check every joint
  std::deque<Sample> pos_hist;                               // at most 4000, like the Python deque
  std::deque<std::pair<double, double>> lag_hist;            // (receive time, header-stamp lag), at most 2000
  std::map<std::string, double> eff_sat_since;

  int index_of(const std::string & n) {
    for (size_t i = 0; i < names.size(); ++i)
      if (names[i] == n) return (int)i;
    names.push_back(n);
    return (int)names.size() - 1;
  }
};

inline void feed_joints(State & s, double now, const std::vector<std::string> & names, const std::vector<double> & pos,
                        const std::vector<double> & vel, const std::vector<double> & eff,
                        std::optional<double> lag = std::nullopt) {
  s.joint_t = now;
  if (lag && std::isfinite(*lag)) {
    s.lag_hist.emplace_back(now, *lag);
    if (s.lag_hist.size() > 2000) s.lag_hist.pop_front();
  }
  Sample smp{now, {}};
  for (size_t i = 0; i < names.size(); ++i) {
    auto g = [&](const std::vector<double> & a) { return i < a.size() ? a[i] : NaN; };
    int k = s.index_of(names[i]);
    s.joints[names[i]] = {g(pos), g(vel), g(eff)};
    if ((int)smp.pos.size() <= k) smp.pos.resize(k + 1, NaN);
    smp.pos[k] = g(pos);
  }
  s.pos_hist.push_back(std::move(smp));
  if (s.pos_hist.size() > 4000) s.pos_hist.pop_front();
}

// Per-joint position noise (one sigma, rad) from the fourth difference over the last noise_window seconds.
inline std::map<std::string, double> estimate_noise(const State & s, double now, const Config & c) {
  std::vector<const Sample *> win;
  for (auto & p : s.pos_hist)
    if (p.t >= now - c.noise_window) win.push_back(&p);
  std::map<std::string, double> out;
  if ((int)win.size() < std::max(c.noise_min_samples, 6)) return out;
  std::vector<double> dts;
  for (size_t i = 0; i + 1 < win.size(); ++i) dts.push_back(win[i + 1]->t - win[i]->t);
  std::sort(dts.begin(), dts.end());
  if (dts.empty() || dts[dts.size() / 2] > c.noise_max_dt) return out;
  const Sample & last = *win.back();
  for (size_t k = 0; k < last.pos.size(); ++k) {
    const std::string & n = s.names[k];
    if (s.arm_joints && !s.arm_joints->count(n)) continue;
    std::vector<double> xs;
    for (auto * p : win) xs.push_back(k < p->pos.size() ? p->pos[k] : NaN);
    double acc = 0;
    int cnt = 0;
    for (size_t i = 0; i + 4 < xs.size(); ++i) {
      double a = xs[i], b = xs[i + 1], d = xs[i + 2], e = xs[i + 3], g = xs[i + 4];
      if (!(std::isfinite(a) && std::isfinite(b) && std::isfinite(d) && std::isfinite(e) && std::isfinite(g))) continue;
      double h = a - 4 * b + 6 * d - 4 * e + g;
      acc += h * h;
      ++cnt;
    }
    if (cnt >= 5) out[n] = std::sqrt(acc / cnt / 70.0);
  }
  return out;
}

inline std::string fmt(const char * f, ...) __attribute__((format(printf, 1, 2)));
inline std::string fmt(const char * f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

inline std::vector<Finding> evaluate(State & s, double now, const Config & c = Config()) {
  std::vector<Finding> f;
  const double up = now - s.t0;
  const bool late = up > c.grace;
  if (!s.joint_t) {
    if (late) f.push_back({ERROR, "JOINT_STATES_NONE", "no /joint_states received (joint_state_broadcaster not running, or the sim is paused)"});
  } else if (now - *s.joint_t > c.stale) {
    f.push_back({ERROR, "JOINT_STATES_STALE", fmt("/joint_states silent for %.1fs (controller manager or joint_state_broadcaster is down)", now - *s.joint_t)});
  }
  const bool live = s.joint_t && now - *s.joint_t <= c.stale;
  if (live) {
    for (auto & [n, pve] : s.joints) {
      if (s.arm_joints && !s.arm_joints->count(n)) continue;
      const double p = pve[0], v = pve[1], e = pve[2];
      if (!std::isfinite(p)) {
        f.push_back({ERROR, "JOINT_NAN", n + " position is NaN/inf (diverged controller or bad state)"});
        continue;
      }
      auto it = s.limits.find(n);
      if (it == s.limits.end()) continue;
      const Limit & lim = it->second;
      if (lim.lower && lim.upper && !lim.continuous && *lim.upper > *lim.lower) {
        const double lo = *lim.lower, hi = *lim.upper, m = c.near_limit * (hi - lo);
        if (p < lo || p > hi) {
          f.push_back({ERROR, "LIMIT_POS", fmt("%s at %.3f is outside its limits [%.3f, %.3f]", n.c_str(), p, lo, hi)});
        } else if (p < lo + m || p > hi - m) {
          f.push_back({WARN, "NEAR_LIMIT", fmt("%s at %.3f is within %.0f%% of its limit [%.3f, %.3f]; planners often fail here", n.c_str(), p, c.near_limit * 100, lo, hi)});
        }
      }
      if (lim.velocity && *lim.velocity != 0.0 && std::isfinite(v) && std::fabs(v) > *lim.velocity * c.vel_margin)
        f.push_back({WARN, "LIMIT_VEL", fmt("%s moving at %.2f rad/s, limit %.2f", n.c_str(), v, *lim.velocity)});
      if (lim.effort && *lim.effort != 0.0 && std::isfinite(e) && std::fabs(e) >= c.effort_frac * *lim.effort) {
        double t0 = s.eff_sat_since.emplace(n, now).first->second;
        if (now - t0 >= c.effort_time)
          f.push_back({WARN, "EFFORT_SAT", fmt("%s pinned at %.1f of %.1f N*m for %.0fs (blocked by contact, or payload too heavy)", n.c_str(), std::fabs(e), *lim.effort, now - t0)});
      } else {
        s.eff_sat_since.erase(n);
      }
    }
  }
  if (live) {
    auto noise = estimate_noise(s, now, c);
    std::string worst;
    double wv = -1;
    int count = 0;
    for (auto & [n, v] : noise)
      if (v > c.noise_rad) {
        ++count;
        if (v > wv) { wv = v; worst = n; }
      }
    if (count)
      f.push_back({WARN, "NOISY_JOINTS", fmt("%s position jitter is about %.4f rad%s, above %g rad (encoder or driver noise, or a loose connection)", worst.c_str(), wv, count > 1 ? fmt(" (and %d more)", count - 1).c_str() : "", c.noise_rad)});
    std::vector<double> recent;
    for (auto & [t, l] : s.lag_hist)
      if (t >= now - c.lag_window) recent.push_back(l);
    std::sort(recent.begin(), recent.end());
    if (recent.size() >= 3 && recent[recent.size() / 2] > c.lag_max)
      f.push_back({WARN, "STREAM_LAG", fmt("/joint_states arrive %.2fs after their header stamp (limit %gs): controllers and planners see old data", recent[recent.size() / 2], c.lag_max)});
  }
  return f;
}

}  // namespace armdoc
