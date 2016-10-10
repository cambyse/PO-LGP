#include "simulation.h"

using namespace std;


simulation::simulation()
{
    Q.resize(state_space[0],state_space[1],n_actions);
    Q.setZero();

    e.resize(state_space[0],state_space[1],n_actions);
    e.setZero();

    initial_position_marker={ position_space[0]/2, position_space[1]/2 };
    initial_position_robot={ 0 , 0 };
    initial_state = {initial_position_marker(0)-initial_position_robot(0), initial_position_marker(1)-initial_position_robot(1)}; 
}


int simulation::e_greedy(intA state_now, double eps) {

    double x = mlr::rnd.uni();
    double q;
    double q_tmp;
    int chosen_action;
    intA candidates;

    for (int i = 0; i < n_actions; i++) {

        if (i == 0) {
            q = Q(state_now(0)+position_space[0]-1, state_now(1)+position_space[1]-1,i);
            candidates = {i};
            continue;
        }

        q_tmp = Q(state_now(0)+position_space[0]-1, state_now(1)+position_space[1]-1,i);
        if (q_tmp > q) {
            candidates = {i};
            q = q_tmp;
        } else if (q_tmp == q) {
            candidates.append(i);
        }
    }

    // random tie breaking
    candidates.permuteRandomly();
    greedy_action = candidates.first();

    if (x >= eps){
    chosen_action = candidates.first();
    }
    if (x < eps) {
        x = mlr::rnd.uni();
        if (x < 0.25) chosen_action=0;
        else if (x < 0.5) chosen_action=1;
        else if (x < 0.75) chosen_action=2;
        else chosen_action=3;
    }
    if (Q(state_now(0)+position_space[0]-1, state_now(1)+position_space[1]-1,candidates.first()) == Q(state_now(0)+position_space[0]-1, state_now(1)+position_space[1]-1,chosen_action));
    {greedy_action = chosen_action; }
    return chosen_action;
}


void simulation::state_transition() {

    intA old_positions_robot = position_robot;

    switch(action) {
    case 0:
        position_robot(1)++;
        break;
    case 1:
        position_robot(0)++;
        break;
    case 2:
        position_robot(1)--;
        break;
    case 3:
        position_robot(0)--;
        break;
    }

    if (position_robot(0) < 0 or
            position_robot(0) >= position_space[0] or
            position_robot(1) < 0 or
            position_robot(1) >= position_space[1]) {
        position_robot=old_positions_robot;
    }

    new_state={position_marker(0)-position_robot(0), position_marker(1)-position_robot(1)};
}


double simulation::get_reward() {
    return (-sqrt(pow(new_state(0), 2) + pow(new_state(1), 2)));
}


void simulation::random_marker_walk() {
    intA new_marker_positions = position_marker;
    double x = mlr::rnd.uni();
    if (x < 0.25) new_marker_positions(0)++;
    else if (x < 0.5) new_marker_positions(1)++;
    else if (x < 0.75) new_marker_positions(0)--;
    else new_marker_positions(1)--;
    if (!(new_marker_positions(0) < 0 or
            new_marker_positions(0) >= position_space[0] or
            new_marker_positions(1) < 0 or
            new_marker_positions(1) >= position_space[1])) {
        position_marker = new_marker_positions;
        state(0) = position_marker(0)-position_robot(0);
        state(1) = position_marker(1)-position_robot(1);
    }
}


void simulation::write_Q_values() {
    ofstream output;
    output.open("dataQvalues", ofstream::out | ofstream::trunc);
    for (int i = 0; i < state_space[0]; i++) {
        output << endl; output << endl; output << endl;
        for (int j = 0; j < state_space[1]; j++) {
            output << endl;
            for (int k = 0; k < n_actions; k++) {
                output << Q(i,j,k) << ' ';
            }
        }
    }
    output.flush();
    output.close();
}


void simulation::run() {

    ofstream out;
    out.open("output");

    for (int p = 0; p < number_of_episodes; p++) {

        e.setZero();
        state = initial_state;
        position_marker=initial_position_marker;
        position_robot=initial_position_robot;
        action = e_greedy(state, epsilon);


        double delta;
        double summed_reward = 0.0;

        for (int step = 0; step < number_of_steps; step++) {

            state_transition();
            reward = get_reward();
            summed_reward += reward;
            next_action = e_greedy(new_state, epsilon);
            delta = reward + gamma*Q(new_state(0)+position_space[0]-1,new_state(1)+position_space[1]-1,greedy_action)-Q(state(0)+position_space[0]-1,state(1)+position_space[1]-1,action);
            e(state(0)+position_space[0]-1,state(1)+position_space[1]-1,action) += 1.0;
            for (int i = 0; i < state_space[0]; i++) {
                for (int j = 0; j < state_space[1]; j++) {
                    for (int m = 0; m < n_actions; m++) {
                        Q(i,j,m) += alpha*delta*e(i,j,m);
                        if (next_action == greedy_action) {
                            e(i,j,m) *= gamma*lambda;
                        } else {
                            e(i,j,m) = 0.0;
                        }
                    }
                }
            }

            state = new_state;
            action = next_action;

            if ( step % 2 == 0 ){

            random_marker_walk();}

        }
        cout << "episode: " << p << " | summed reward: " << summed_reward << endl;
        out << p << ' ' << summed_reward << endl;

    }
    gnuplot("plot 'output' us 1:2 with lines smooth sbezier", true);
    out.close();

    write_Q_values();
}

