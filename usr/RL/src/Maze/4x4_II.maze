maze_t(
    QString("4x4_II"),
    4,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(4,4,1,1)),
    reward_ptr_t(new reward_t({0,1},0)),
    vector<wall_t>({
            {0,4},
            {4,5},
            {8,12},
            {14,15},
            {5,6},
            {2,3}
        }),
    vector<maze_reward_t>({
            { 0, 8, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 1, 4, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 8,13, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            {12,14, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            {14,11, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            {15,10, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            {11, 9, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 9, 6, 3, 1, ON_RELEASE_NO_PUNISH, 255,   0,   0},
            { 5, 7, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 7, 3, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 7, 2, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
            { 2, 1, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0}
        }),
    vector<door_t>({
            door_t(observation_t(4,4, 0),observation_t(4,4, 4),observation_t(4,4, 1),    UP_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4, 8),observation_t(4,4,12),observation_t(4,4, 4), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4,14),observation_t(4,4,15),observation_t(4,4,14),  DOWN_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4, 5),observation_t(4,4, 6),observation_t(4,4, 5),  LEFT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
                door_t(observation_t(4,4, 3),observation_t(4,4, 2),observation_t(4,4, 3), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) )
                })
    )
