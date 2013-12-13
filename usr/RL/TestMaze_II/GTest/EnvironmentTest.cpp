#include <gtest/gtest.h>

#include <vector>

#include "../PredictiveEnvironment.h"
#include "../Maze/Maze.h"
#include "MinimalEnvironmentExample/MinimalEnvironment.h"

TEST(EnvironmentTest, Minimal) {
    std::vector<std::shared_ptr<PredictiveEnvironment> > environments;
    environments.push_back(new MinimalEnvironment());
}

