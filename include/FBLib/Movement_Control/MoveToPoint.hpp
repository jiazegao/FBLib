struct MoveDistanceParams {
  bool forwards = true;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches

};
struct MoveToPointParams {
  bool forwards = true;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches
};