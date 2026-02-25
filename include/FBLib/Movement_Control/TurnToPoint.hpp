enum class TurnDirection {
  CCWCOUNTERCLOCKWISE,
  CWCLOCKWISE,
  SHORTEST
};
struct TurntoHeadingParams {
  TurnDirection direction = TurnDirection::SHORTEST;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches
};
struct TurnToPointParams {
  TurnDirection direction = TurnDirection::SHORTEST;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches
};