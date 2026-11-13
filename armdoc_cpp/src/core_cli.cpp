// Feed a recorded scenario to the C++ core and print the findings. Used by the equivalence test against Python.
// Scenario lines:  L name lower upper velocity effort continuous   (use "-" for none)
//                  A name                                          (restrict the checks to these joints)
//                  S now lag n name pos vel eff ...                (lag "-" for none)
//                  E now                                           (evaluate; prints "level code" lines then "--")
#include <fstream>
#include <iostream>
#include <sstream>
#include "armdoc_cpp/core.hpp"
using namespace armdoc;

static std::optional<double> opt(const std::string & s) { return s == "-" ? std::nullopt : std::optional<double>(std::stod(s)); }

int main(int argc, char ** argv) {
  if (argc != 2) { std::fprintf(stderr, "usage: armdoc_core_cli scenario.txt\n"); return 2; }
  std::ifstream in(argv[1]);
  State s;
  Config c;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream is(line);
    std::string tag;
    is >> tag;
    if (tag == "L") {
      std::string n, lo, hi, v, e;
      int cont;
      is >> n >> lo >> hi >> v >> e >> cont;
      s.limits[n] = Limit{opt(lo), opt(hi), opt(v), opt(e), cont != 0};
    } else if (tag == "A") {
      std::string n;
      if (!s.arm_joints) s.arm_joints = std::set<std::string>();
      is >> n;
      s.arm_joints->insert(n);
    } else if (tag == "S") {
      double now;
      std::string lag;
      int n;
      is >> now >> lag >> n;
      std::vector<std::string> names;
      std::vector<double> p, v, e;
      for (int i = 0; i < n; ++i) {
        std::string nm, a, b, d;       // read as text: istream >> double cannot parse "nan" or "inf"
        is >> nm >> a >> b >> d;
        names.push_back(nm); p.push_back(std::stod(a)); v.push_back(std::stod(b)); e.push_back(std::stod(d));
      }
      feed_joints(s, now, names, p, v, e, opt(lag));
    } else if (tag == "E") {
      double now;
      is >> now;
      for (auto & f : evaluate(s, now, c)) std::cout << f.level << ' ' << f.code << '\n';
      std::cout << "--\n";
    }
  }
  return 0;
}
