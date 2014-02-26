maze_t(
    QString("Wrong"),
    1,
    action_ptr_t(new action_t("stay")),
    observation_ptr_t(new observation_t(2,2,0,0)),
    reward_ptr_t(new reward_t({-1,0,1,2},1)),
    vector<wall_t>({
//                { 0, 3},
//                { 3, 4}
        }),
    vector<maze_reward_t>({
//                { 0, 1, 1,  3, ON_RELEASE_NO_PUNISH,   0, 200, 200},
            { 0, 1, 1,  2, EACH_TIME_NO_PUNISH,   0, 200, 200},
            { 0, 1, 1, -1, ON_RELEASE_PUNISH_FAILURE,   0, 200, 200},
//                { 0, 1, 1,  2, EACH_TIME_PUNISH_FAILURE,   0, 200, 200}
                }),
    vector<door_t>({
//                door_t(observation_t(2,2,0,0), observation_t(2,2,1,1), observation_t(2,2,0,0),  PASS_BUTTON, 0, color_t(1.0,0.5,0.0) ),
//                door_t(observation_t(2,3,0,0), observation_t(2,2,0,1), observation_t(2,2,0,0), RIGHT_BUTTON, 5, color_t(1.0,0.0,1.0) )
        })
    )
