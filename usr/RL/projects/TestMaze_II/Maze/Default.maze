maze_t(
    QString("Default"),
    2,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(2,2,0,0)),
    reward_ptr_t(new reward_t({0,1},0)),
    vector<wall_t>({
            { 0, 1}
        }),
    vector<maze_reward_t>({
            { 0, 3, 2, 1, EACH_TIME_NO_PUNISH,   0,   0,   0},
            { 0, 1, 1, 1, EACH_TIME_NO_PUNISH,   0,   0,   0}
        }),
    vector<door_t>({
            door_t(observation_t(2,2,0,0), observation_t(2,2,1,0), observation_t(2,2,0,0), LEFT_BUTTON, -1, color_t(0.0,0.8,0.0) )
                })
    )
