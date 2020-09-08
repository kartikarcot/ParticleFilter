#include <MotionModel.hpp>

MotionModel::MotionModel(double rot1_var, double trans_var, double rot2_var) 
: process_noise(std::move(OdomModelNoise(rot1_var, trans_var, rot2_var))){}


void MotionModel::predictOdometryModel(Particle& p, Pose2D& u_t0, Pose2D& u_t1)
{
    auto rot1 = atan2( u_t1.y-u_t0.y , u_t1.x-u_t0.x ) - u_t0.theta;
    auto trans = pow((u_t1.y-u_t0.y),2.0) + pow((u_t1.x-u_t0.x),2.0);
    auto rot2 = u_t1.theta - rot1 - u_t0.theta ;

    //Conceptual doubt : to add or subtract
    std::default_random_engine generator2;
    auto rot1_bar = rot1 - process_noise.dists[0](generator2);
    auto trans_bar = trans - process_noise.dists[0](generator2);
    auto rot2_bar = rot2 - process_noise.dists[0](generator2);
    
    p.pose.x = p.pose.x + trans_bar * cos( rot1_bar + p.pose.theta) ;
    p.pose.y = p.pose.y + trans_bar * sin( rot1_bar + p.pose.theta) ;
    p.pose.theta += rot1_bar + rot2_bar ;


}
