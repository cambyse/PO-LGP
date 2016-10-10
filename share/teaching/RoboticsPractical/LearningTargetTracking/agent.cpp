#include "agent.h"


arr agent::choose_action(arr f, arr theta_x, arr theta_y, double sigma, bool useRos){
    double action_x = (~theta_x*f)(0) + sqrt(sigma)*mlr::rnd.uni();
    double action_y = (~theta_y*f)(0) + sqrt(sigma)*mlr::rnd.uni();
    double target_x = state(0) + action_x;
    double target_y = state(1) + action_y;
    if(useRos){
        /*
        if(target_x < state_space(0)){
            action_x = state_space(0) - state(0);
        } else if (target_x > state_space(1) ){
            action_x = state_space(1) - state(0);
        }
        if(target_y < state_space(2)){
            action_y = state_space(2) - state(1);
        } else if (target_y > state_space(3) ){
            action_y = state_space(3) - state(1);
        }
        */
        if(target_x < state_space(0) || target_x > state_space(1)) {
            action_x = (state_space(1)-state_space(0)-0.05) * mlr::rnd.uni() + state_space(0) - state(0) + 0.025;
        }

        if(target_y < state_space(2) || target_y > state_space(3)) {
            action_y = (state_space(3)-state_space(2)-0.05) * mlr::rnd.uni() + state_space(2) - state(1) + 0.025;
        }
    }
    return ARR(action_x, action_y);
}

arr agent::compute_td_error(arr reward, arr f1, arr w_x, arr w_y, arr f){
    double delta_x = reward(0) + gamma*(~f1*w_x)(0) - (~f*w_x)(0);
    double delta_y = reward(1) + gamma*(~f1*w_y)(0) - (~f*w_y)(0);
    return ARR(delta_x, delta_y);
}

void agent::critic_update(arr &w_x, arr &w_y, arr delta, arr f){
    w_x += f*delta(0)*alpha;
    w_y += f*delta(1)*alpha;
}

void agent::actor_update(arr &theta_x, arr &theta_y, arr f, arr w_x, arr w_y){
    theta_x += beta*f*~f*w_x;
    theta_y += beta*f*~f*w_y;
}

void agent::learn(environment &env, bool useRos){

    double threshold;
    if (useRos) {
        threshold = 0.02;
    } else {
        threshold = 0.01;
    }
    cout << "alpha: " << alpha << endl;
    cout << "beta: " << beta << endl;
    int num_of_steps = 0;
    int number_of_learning_episodes = 10;
    arr theta_old_x = ARR(10.,10.);
    arr theta_old_y = ARR(10.,10.);
    summed_reward = ARR(0.,0.);
    state = env.arm_position;
    state_space = env.state_space;
    cout << "learning" << endl;
    for (int i = 0; i < number_of_learning_episodes; i++) {
        //for(int j = 0; j < 1000; j++){
        while(sqrDistance(theta_old_x, theta_x)+ sqrDistance(theta_old_y, theta_y) > pow(10, -14)){
            if(useRos){
                env.track_marker_position_baxter();
            }
            f = state-env.marker_position;
            cout << "theta: " << theta_x << ' ' << theta_y << endl;

            action = choose_action(f, theta_x, theta_y, 0.01, useRos);
            cout << "action: " << action << state << endl;
            cout << "step " << num_of_steps+1 << endl;
            if(useRos){
                state = env.get_next_state_baxter(state, action);
                cout << state << endl;
            }else{
                state = env.get_next_state(state, action);
            }
            reward = env.get_reward(state, threshold);
            cout << "reward: " << reward << endl;
            summed_reward += reward;
            f1 = state - env.marker_position;
            delta = compute_td_error(reward, f1, w_x, w_y, f);
            critic_update(w_x, w_y, delta, f);
            theta_old_x = theta_x;
            theta_old_y = theta_y;
            actor_update(theta_x, theta_y, f, w_x, w_y);
            cout << "-----------------------------------------------" << endl;
            num_of_steps++;
        }
        if(useRos){
            break;
        }
    }
    cout << "theta_x: " << theta_x << endl;
    cout << "theta_y: " << theta_y << endl;
    cout << "learning finish" << endl;

}


void agent::test(environment &env, bool useRos){
    double threshold;
    if (useRos) {
        threshold = 0.02;
    } else {
        threshold = 0.01;
    }
    int num_of_steps = 0;
    int number_of_test_episodes = 1000;
    ofstream file;
    if(!useRos){
        file.open("data_test_plot");
    }

    num_of_steps = 0;
  // theta_x=ARR(-1,0);
  // theta_y=ARR(0,-1);
    for (int i = 0; i < number_of_test_episodes; i++) {
        if(!useRos){
            env.set_random_positions();
        }
        state = env.arm_position;
        summed_reward = ARR(0.0, 0.0);
        num_of_steps = 0;
        while (sqrDistance(state, env.marker_position) > pow(10, -14)) {
            if (useRos) env.track_marker_position_baxter();
            f = state-env.marker_position;
            //action = choose_action(f, theta_x, theta_y, 0.0);
            action = ARR((~theta_x*f)(0), (~theta_y*f)(0));
            if (useRos) {
                state= env.get_next_state_baxter(state, action);
            } else {
                state = env.get_next_state(state, action);
                reward = env.get_reward(state, threshold);
                summed_reward += reward;
                num_of_steps++;
            }
        }
        if(useRos){
            i = 0;
        }else {
            file << i << ' ' << num_of_steps << ' ' << (summed_reward(0) + summed_reward(1)) << ' ' <<  (summed_reward(0) + summed_reward(1)) / num_of_steps << endl;
        }
    }

    if(!useRos){
        gnuplot("plot 'data_test_plot' us 1:2 smooth bezier", true);
        gnuplot("plot 'data_test_plot' us 1:3 smooth bezier", true);
        gnuplot("plot 'data_test_plot' us 1:4 smooth bezier", true);
        file.close();
    }

}

void agent::writeParameter(bool useRos){
    ofstream file;
    if(useRos){
        file.open("parameter_ros");
    }else{
        file.open("parameter_sim");
    }
    file << theta_x(0) << ' ' << theta_x(1) << ' ' << theta_y(0) << ' ' << theta_y(1);

    file.close();
}

void agent::readParameter(bool fromRos){
    char dummy;
    ifstream file;
    if(fromRos){
        file.open("parameter_ros");
    }else{
        file.open("parameter_sim");
    }
    file >> theta_x(0) >> dummy >> theta_x(1) >> dummy >> theta_y(0) >> dummy >> theta_y(1);

    file.close();
}
