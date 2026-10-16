#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <cstdio>
#include <chrono>


// TODO: we can apply the robot's current momentum/speed * time it usually takes, to get a even more precise answer.
//
constexpr float AIR_DENSITY {1.2f};
constexpr float DRAG_COEFFICIENT {0.45f};
constexpr float BALL_AREA {0.0043f};
constexpr float BALL_MASS {0.026f};
constexpr float DRAG_K {AIR_DENSITY * DRAG_COEFFICIENT * BALL_AREA / (2.0f * BALL_MASS)};

constexpr float TIME_INTERVAL {0.02};
constexpr float DEPTH_LIMIT {100};
constexpr float DEADZONE {0.2};

float angleFunc(float horizontalDistance) {
  return (-0.714 * horizontalDistance + 43.571) * M_PI / 180.0f;
}

// float angleFunc(float horizontalDistance) {
//   return 0.8;
// }

std::array<float, 2> basketLocation {10, 1.5};

std::array<float, 2> normalize(const std::array<float, 2>& vec) {
  float length {std::sqrt(vec[0] * vec[0] + vec[1] * vec[1])};
  if (length == 0.0f) {
    return vec;
  }
  return {vec[0] / length, vec[1] / length};
}

std::array<float, 2> getVctFromSettings(const float& magnitude, const float& angle) {
  std::array<float, 2> vct {};
  vct[0] = magnitude * cosf(angle);
  vct[1] = magnitude * sinf(angle);

  return vct;
}

// cycle speed is 50ms
void nextLocation(std::array<float, 2>& thisLocation, std::array<float, 2>& vLast, int& cycle) {
  std::array<float, 2> gAccel {0, -9.8};
  std::array<float, 2> aaAccel {};
  std::array<float, 2> accel {};
  float speed {std::sqrt(vLast[0] * vLast[0] + vLast[1] * vLast[1])};
  for (std::size_t i = 0; i < 2; ++i) {
    aaAccel[i] = -DRAG_K * speed * vLast[i];
  }

  for (std::size_t i = 0; i < 2; ++i) {
    accel[i] = aaAccel[i] + gAccel[i]; 
  }
  
  std::array<float, 2> allV {};
  for (std::size_t i = 0; i < 2; ++i) {
    allV[i] = accel[i] * TIME_INTERVAL + vLast[i];
  }
  vLast = allV;

  std::array<float, 2> finalLocation {};

  for (std::size_t i = 0; i < 2; ++i) {
    finalLocation[i] = thisLocation[i] + allV[i] * TIME_INTERVAL; 
  }
  
  ++cycle;
  thisLocation = finalLocation;

  printf("the next vector [%.3f, %.3f]  |  time %.2f\n", finalLocation[0], finalLocation[1], cycle * TIME_INTERVAL);
}

void binarySearch(const std::array<float, 2>& startV, float lo = 0.0f, float hi = 2.0f, int depth = 0) {
  float scale {(lo + hi) / 2};
  std::array<float, 2> lastLoc {0, 0};
  std::array<float, 2> lastV {startV[0] * scale, startV[1] * scale};
  int cycle = 0;

  for (std::size_t o = 0; o < 100; ++o) {
    if (lastLoc[0] >= basketLocation[0] || lastLoc[1] < 0) {
      break;
    }
    nextLocation(lastLoc, lastV, cycle);
  }

  if (lastLoc[1] > basketLocation[1] - DEADZONE && lastLoc[1] < basketLocation[1] + DEADZONE) {
    printf("YOU GOT IT MAN: starting velocity vector: [%.2f, %.2f]\n", startV[0] * scale, startV[1] * scale);
  } else if (depth >= DEPTH_LIMIT) {
    printf("no solution found\n");
  } else if (lastLoc[1] > basketLocation[1]) {
    binarySearch(startV, lo, scale, depth + 1);
  } else {
    binarySearch(startV, scale, hi, depth + 1);
  }
}

void test() {
  std::array<float, 2> lastLoc {0.0,0.0};
  std::array<float, 2> lastV {0.0, 2.0};
  int cycle {0};

  for (std::size_t i = 0; i < 100; ++i) {
    if (lastLoc[1] < 0) {
      break;
    }
    nextLocation(lastLoc, lastV, cycle);
  }
}

int main() {
  std::array<float, 2> lastLoc {0.0,0.0};
  int cycle {0};
  float mag {};
  
  printf("enter the basketLocation X: ");
  std::cin >> basketLocation[0];
  printf("enter the basketLocation Y: ");
  std::cin >> basketLocation[1];
  printf("enter the magnitude: ");
  std::cin >> mag;

  auto start = std::chrono::steady_clock::now();
  std::array<float, 2> lastV = getVctFromSettings(mag, angleFunc(basketLocation[0]));
  binarySearch(lastV);
  auto stop = std::chrono::steady_clock::now();

  auto diff = stop - start;
  auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
  printf("total time elapsed: %lldms\n", diff_ms.count());

  return 0;
}
