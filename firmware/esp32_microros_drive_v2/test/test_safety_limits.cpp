// PCで実行: g++ -std=c++17 -I include test/test_safety_limits.cpp -o /tmp/t && /tmp/t
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "safety_limits.h"

int main() {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(sanitizeCommand(0.5, 1.2) == 0.5);
  assert(sanitizeCommand(-0.5, 1.2) == -0.5);
  assert(sanitizeCommand(9.0, 1.2) == 1.2);
  assert(sanitizeCommand(-9.0, 1.2) == -1.2);
  assert(sanitizeCommand(nan, 1.2) == 0.0);
  assert(sanitizeCommand(inf, 1.2) == 0.0);
  assert(sanitizeCommand(-inf, 1.2) == 0.0);

  const float ok[12] = {6, 1, 0, 6, 0, 0, 6.1f, 1, 0, 6.1f, 0, 0};
  assert(arePidGainsValid(ok, 12));
  float bad[12];
  for (int i = 0; i < 12; i++) bad[i] = ok[i];
  bad[5] = -0.1f; assert(!arePidGainsValid(bad, 12));
  bad[5] = 0; bad[7] = 1000.0f; assert(!arePidGainsValid(bad, 12));
  bad[7] = 1; bad[3] = (float)nan; assert(!arePidGainsValid(bad, 12));
  bad[3] = (float)inf; assert(!arePidGainsValid(bad, 12));
  bad[3] = 200.0f; assert(arePidGainsValid(bad, 12));
  assert(!arePidGainsValid(nullptr, 12));
  assert(!arePidGainsValid(ok, 0));
  std::puts("OK");
  return 0;
}
