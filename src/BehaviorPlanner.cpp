//
// Created by Bob Lutz on 7/22/18.
//

#include "BehaviorPlanner.h"

BehaviorPlanner::BehaviorPlanner() = default; // Initializes Vehicle

BehaviorPlanner::BehaviorPlanner(int lane, string state) {
  this->lane = lane;
  this->state = state;

//  max_acceleration = -1;
}

BehaviorPlanner::~BehaviorPlanner() = default;


