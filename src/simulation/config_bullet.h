#pragma once
#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct BulletConfig : Config {
  static btVector3 gravity;
  static float dt;
  static int maxSubSteps;
  static float internalTimeStep;
  static float friction;
  static float restitution;
  static float margin;
  static float linkPadding;
  static float timeStep;
  static float fixedTimeStep;

  BulletConfig() : Config() {
    params.push_back(new Parameter<float>("gravity", &gravity.m_floats[2], "gravity (z component)")); 
    params.push_back(new Parameter<float>("dt", &dt, "timestep for fixed-step simulations")); 
    params.push_back(new Parameter<int>("maxSubSteps", &maxSubSteps, "maximum Bullet internal substeps per simulation step"));
    params.push_back(new Parameter<float>("timeStep", &timeStep, "the amount of time to step the simulation by. Typically you're going to be passing it the time since you last called it"));
    params.push_back(new Parameter<float>("fixedTimeStep", &fixedTimeStep, "mtimeStep < maxSubSteps * fixedTimeStep"));
    params.push_back(new Parameter<float>("internalTimeStep", &internalTimeStep, "internal Bullet timestep"));
    params.push_back(new Parameter<float>("friction", &friction, "default friction coefficient for rigid bodies"));
    params.push_back(new Parameter<float>("restitution", &restitution, "not currently implemented"));
    params.push_back(new Parameter<float>("margin", &margin, "not currently implemented"));
    params.push_back(new Parameter<float>("linkPadding", &linkPadding, "expand links by that much if they're convex hull shapes"));
  }
};

#define DT BulletConfig::dt
