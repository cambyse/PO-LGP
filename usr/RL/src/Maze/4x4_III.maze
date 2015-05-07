maze_t(
    QString("4x4_III"),
    3,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(4,4,1,1)),
    reward_ptr_t(new reward_t({0,1},0)),
    vector<wall_t>({
            { 5, 9},
            {12,13},
            {11,15},
            { 5, 1},
            { 5, 4},
            { 2, 6},
            { 3, 7},
            { 9,10},
            {10,14}
        }),
    vector<maze_reward_t>({
            { 0, 8, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            { 8,13, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            { 9,12, 3, 1,  EACH_TIME_NO_PUNISH, 255, 100,   0},
            {12,14, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            {13,15, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            {11, 6, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            {10, 5, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            { 5, 2, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            { 1, 3, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
            { 2, 0, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0}
        }),
    vector<door_t>({
            door_t(observation_t(4,4,12),observation_t(4,4,13),observation_t(4,4,13),  DOWN_BUTTON, -2, color_t(0.0,1.0,0.0) ),
                door_t(observation_t(4,4,15),observation_t(4,4,11),observation_t(4,4,15), RIGHT_BUTTON, -2, color_t(0.0,1.0,0.0) ),
                door_t(observation_t(4,4, 5),observation_t(4,4, 1),observation_t(4,4, 5),  DOWN_BUTTON, -2, color_t(0.0,1.0,0.0) )
                })
    )
