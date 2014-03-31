maze_t(
    QString("4x4_I"),
    4,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(4,4,1,1)),
    reward_ptr_t(new reward_t({0,1},0)),
    vector<wall_t>({
            { 4, 8},
            { 5, 9},
            { 6,10},
            { 7,11},
            { 1, 2},
            { 5, 6},
            { 9,10},
            {13,14},
            { 8, 9},
            { 2, 3},
            { 6, 7},
            {14,15},
            {11,15},
            { 3, 7}
        }),
    vector<maze_reward_t>({
            { 0,  4, 3, 1, ON_RELEASE_NO_PUNISH, 255, 120,  0},
            { 5,  0, 2, 1, ON_RELEASE_NO_PUNISH, 200,   0,  0},
            { 1,  5, 3, 1, ON_RELEASE_NO_PUNISH, 255, 120,  0},
            { 2,  1, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
            { 7,  6, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
            {11,  7, 4, 1,  EACH_TIME_NO_PUNISH, 255, 200,  0},
            {10, 15, 3, 1,  EACH_TIME_NO_PUNISH, 255, 120,  0},
            {13, 14, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
            { 8,  9, 4, 1,  EACH_TIME_NO_PUNISH, 255, 200,  0},
            { 4,  8, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0}
        }),
    vector<door_t>({
            door_t(observation_t(4,4, 4), observation_t(4,4, 8), observation_t(4,4, 4),  DOWN_BUTTON,  0, color_t(0.6,0.0,1.0) ),
                door_t(observation_t(4,4, 8), observation_t(4,4, 9), observation_t(4,4, 8),  LEFT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
                door_t(observation_t(4,4,13), observation_t(4,4,14), observation_t(4,4,13), RIGHT_BUTTON,  0, color_t(0.6,0.0,1.0) ),
                door_t(observation_t(4,4,14), observation_t(4,4,15), observation_t(4,4,10),    UP_BUTTON, -3, color_t(0.0,0.5,0.0) ),
                door_t(observation_t(4,4,15), observation_t(4,4,11), observation_t(4,4,15), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4,11), observation_t(4,4, 7), observation_t(4,4,11),    UP_BUTTON,  0, color_t(0.6,0.0,1.0) ),
                door_t(observation_t(4,4, 7), observation_t(4,4, 3), observation_t(4,4, 7), RIGHT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
                door_t(observation_t(4,4, 3), observation_t(4,4, 7), observation_t(4,4, 3), RIGHT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
                door_t(observation_t(4,4, 7), observation_t(4,4, 6), observation_t(4,4, 3),  LEFT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4, 2), observation_t(4,4, 1), observation_t(4,4, 2),  LEFT_BUTTON,  0, color_t(0.6,0.0,1.0) )
                })
    )
