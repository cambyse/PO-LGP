maze_t(
    QString("3x3"),
    6,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(3,3,1,1)),
    reward_ptr_t(new reward_t({0,1,3,8},0)),
    vector<wall_t>({
            { 0, 1},
            { 0, 3},
            { 2, 1},
            { 2, 5},
            { 6, 3},
            { 6, 7},
            { 8, 7},
            { 8, 5}
        }),
    vector<maze_reward_t>({
            { 3, 5, 4, 8, ON_RELEASE_NO_PUNISH,   0, 200,   0},
            { 5, 3, 6, 8, ON_RELEASE_NO_PUNISH,   0, 200, 200},
            { 4, 1, 1, 1, ON_RELEASE_NO_PUNISH, 200, 200,   0},
            { 4, 7, 3, 3, ON_RELEASE_NO_PUNISH, 200,   0,   0}
        }),
    vector<door_t>({})
    )
