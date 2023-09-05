#pragma once

class LinearTrajectory {
 public:
  LinearTrajectory(double start, double end, double duration);
  double getPos(double time);
  double getVel(double time);
  double getAcc(double time);

 private:
  double start_;
  double end_;
  double duration_;
};