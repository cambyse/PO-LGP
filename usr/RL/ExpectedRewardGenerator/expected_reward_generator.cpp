#include "expected_reward_generator.h"

#include <fstream>
#include <math.h>
#include <algorithm>
#include <iostream>

ExpectedRewardGenerator::ExpectedRewardGenerator(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}

ExpectedRewardGenerator::~ExpectedRewardGenerator()
{

}

double ExpectedRewardGenerator::get_value(const std::vector<bool>& rewards, const double& minR, const double& maxR, const double& discount) {
    double current_discount = 1;
    double value = 0;
    for(unsigned int idx=0; idx<rewards.size(); ++idx) {
        if(rewards[idx]) {
            value += current_discount*maxR;
        } else {
            value += current_discount*minR;
        }
        current_discount *= discount;
    }
    return value;
}

double ExpectedRewardGenerator::probability(const std::vector<bool>& rewards, const double& pMin, const double& pMax) {
    double prob = 1;
    for(unsigned int idx=0; idx<rewards.size(); ++idx) {
        if(rewards[idx]) {
            prob *= pMax;
        } else {
            prob *= pMin;
        }
    }
    return prob;
}

void ExpectedRewardGenerator::add_density(std::vector<double>& p_values, const double& value, const double& discount, const int& depth, const double& minR, const double& maxR, const double& prob) {
    double step_size = ( (maxR - minR)/(1 - discount) ) / (p_values.size()-1);
    double new_min = value + minR*pow(discount,depth)/(1-discount);
    double new_max = value + maxR*pow(discount,depth)/(1-discount);
    int step_start = (int)round( (1-discount)*new_min/(maxR-minR) * (p_values.size()-1) );
    int step_end = (int)round( (1-discount)*new_max/(maxR-minR) * (p_values.size()-1) );
    double p_sum = 0;
    for(int idx = step_start; idx<=step_end; ++idx) {
        if(idx>=(int)p_values.size()) {
            std::cout << "Error: Index too large" << std::endl;
            break;
        }
        p_values[idx] += prob/( step_size * (step_end-step_start+1) );
        p_sum += prob/( step_size * (step_end-step_start+1) );;
    }
    if(fabs(p_sum*step_size-prob)>1e-10) {
        std::cout << "p_sum*step_size=" << p_sum*step_size << ", prob=" << prob << std::endl;
    }
}

void ExpectedRewardGenerator::minR_changed(int i) {
    double value = i - ui._wMinR->minimum();
    value /= ui._wMinR->maximum()-ui._wMinR->minimum();
    ui._wMinRValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::maxR_changed(int i) {
    double value = i - ui._wMaxR->minimum();
    value /= ui._wMaxR->maximum()-ui._wMaxR->minimum();
    ui._wMaxRValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::maxR_P_changed(int i) {
    double value = i - ui._wPMax->minimum();
    value /= ui._wPMax->maximum()-ui._wPMax->minimum();
    ui._wPMaxValue->setText(QString::number(value));
    ui._wPMinValue->setText(QString::number(1-value));
}

void ExpectedRewardGenerator::resolution_changed(int i) {
    ui._wResolutionValue->setText(QString::number(i));
}

void ExpectedRewardGenerator::depth_changed(int i) {
    ui._wDepthValue->setText(QString::number(i));
}

void ExpectedRewardGenerator::discount_changed(int i) {
    double value = i - ui._wDiscount->minimum();
    value /= ui._wDiscount->maximum()-ui._wDiscount->minimum();
    ui._wDiscountValue->setText(QString::number(value));
}

void ExpectedRewardGenerator::plot() {

    std::fstream plot_file("plot_file.txt", std::fstream::out );

    if(!plot_file.good()) {
        std::cout << "Error opening plot file." << std::endl;
        plot_file.close();
        return;
    }

    double minR, maxR, pMin, pMax, discount;
    int res, depth;

    bool minR_ok, maxR_ok, pMin_ok, pMax_ok, res_ok, depth_ok, discount_ok;

    minR = ui._wMinRValue->text().toDouble(&minR_ok);
    maxR = ui._wMaxRValue->text().toDouble(&maxR_ok);
    pMin = ui._wPMinValue->text().toDouble(&pMin_ok);
    pMax = ui._wPMaxValue->text().toDouble(&pMax_ok);
    res  = ui._wResolutionValue->text().toInt(&res_ok);
    depth  = ui._wDepthValue->text().toInt(&depth_ok);
    discount = ui._wDiscountValue->text().toDouble(&discount_ok);
    if( !(minR_ok && maxR_ok && pMin_ok && pMax_ok && res_ok && depth_ok && discount_ok) ) {
        plot_file << "# ERROR" << std::endl;
        plot_file.close();
        return;
    }

    std::vector<bool> rewards(depth);
    std::vector<double> p_values(res,0);
    double prob_sum = 0;

    // for different overall number of minimum and maximum reward
    for(unsigned int number_of_rewards=0; number_of_rewards<=rewards.size(); ++number_of_rewards) {

        // fill in rewards
        for(unsigned int idx=0; idx<rewards.size(); ++idx) {
            rewards[idx] = idx<number_of_rewards;
        }

        // sort for later permutation
        std::sort(rewards.begin(),rewards.end());

        // go through permutations
        do {

            // calculate value and probability
            double value = get_value(rewards, minR, maxR, discount);
            double prob = probability(rewards, pMin, pMax);

            // add up probability over possible value range
            add_density(p_values, value, discount, depth, minR, maxR, prob);

            // for later check
            prob_sum += prob;

            // write value and probability to file
            plot_file << value << " " << prob << " ( ";

            // write specific reward permutation to file
            for(unsigned int idx=0; idx<rewards.size(); ++idx) {
                plot_file << rewards[idx] << " ";
            }
            plot_file << ")" << std::endl;

        } while(std::next_permutation(rewards.begin(),rewards.end()));
    }

    // check
    if(fabs(prob_sum-1)>1e-10) {
        std::cout << "Error: Probabilities do not sum to one (prob_sum=" << prob_sum << ")" << std::endl;
    }

    // separate data blocks
    plot_file << std::endl;
    plot_file << std::endl;

    // for later check
    prob_sum = 0;
    double step_size = ( (maxR - minR)/(1 - discount) ) / (p_values.size()-1);

    // write unscaled probability density values to file
    double unscaled_mean = 0;
    for(unsigned int idx=0; idx<p_values.size(); ++idx) {
        unscaled_mean += (minR + idx*step_size)*p_values[idx]*step_size;
        plot_file << minR + idx*step_size << " " << p_values[idx] << std::endl;
        prob_sum += p_values[idx];
    }

    std::cout << "Unscaled mean = " << unscaled_mean << std::endl;
    if(fabs(prob_sum*step_size-1)>1e-10) {
        std::cout << "Error: Probability densities do not sum to one (prob_sum=" << prob_sum*step_size << std::endl;
    }

    // separate data blocks
    plot_file << std::endl;
    plot_file << std::endl;

    double scale_offset = minR/(1 - discount);
    double scale_factor = (maxR - minR)/(1 - discount);

    // write scaled probability density values to file
    double scaled_mean = 0;
    for(unsigned int idx=0; idx<p_values.size(); ++idx) {
        scaled_mean += ( (minR + idx*step_size - scale_offset)/scale_factor ) * p_values[idx]*step_size;
        plot_file << (minR + idx*step_size - scale_offset)/scale_factor << " " << p_values[idx]*scale_factor << std::endl;
    }

    std::cout << "Scaled mean = " << scaled_mean << std::endl;

    plot_file.close();
}
