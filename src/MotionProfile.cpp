#include "MotionProfile.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/chassis/chassis.hpp"


double RTMotionProfile::Pose::getDistance(Pose pose) {
    return sqrt(pow(pose.x - this->x, 2) + pow(pose.y - this->y, 2));
}
double RTMotionProfile::Pose::getHeading(Pose pose) {
    return atan2(pose.y - this->y, pose.x - this->x);
}

RTMotionProfile::Bezier::Bezier(const Point &p0, const Point &p1, const Point &p2, const Point &p3, int length)
{
    this->A = p0;
    this->B = p1;
    this->C = p2;
    this->D = p3;
    // this->P << p0.x, p1.x, p2.x, p3.x,
    //     p0.y, p1.y, p2.y, p3.y;
    this->controlPoints << p0.x, p0.y,
        p1.x, p1.y,
        p2.x, p2.y,
        p3.x, p3.y;

    this->len = length;
    this->lengths = std::vector<double>(this->len + 1, 0);

    this->coefficients << -1, 3, -3, 1,
        3, -6, 3, 0,
        -3, 3, 0, 0,
        1, 0, 0, 0;
    this->derivative_coefficients << -3, 9, -9, 3,
        6, -12, 6, 0,
        -3, 3, 0, 0;
    this->second_derivative_coefficients << -6, 18, -18, 6,
        6, -12, 6, 0;
}

double RTMotionProfile::Bezier::tAtArcLength(double arcLength) {
    double target = arcLength;
    double lower_bound = 0;
    double higher_bound = this->len;
    double midpoint_index = 0;

    //implement binary search
    while (lower_bound < higher_bound) {
        midpoint_index = static_cast<int>(lower_bound + (higher_bound - lower_bound)/2);
        if (this->lengths[midpoint_index] < target) {
            lower_bound = midpoint_index + 1;
        } else {
            higher_bound = midpoint_index;
        }
    }
    if (this->lengths[midpoint_index] > target) midpoint_index--;
    double prevLength = this->lengths[midpoint_index];
    if (prevLength == target) return midpoint_index / static_cast<double>(this->len);
    else return (midpoint_index + (target - prevLength) / (this->lengths[midpoint_index+1] - prevLength)) / static_cast<double>(this->len);
}

RTMotionProfile::Point RTMotionProfile::Bezier::getPoint(double t) {
    Eigen::RowVector4d cubicPolynomialPowers;
    cubicPolynomialPowers << t*t*t, t*t, t, 1;
    auto result = (cubicPolynomialPowers * this->coefficients) * this->controlPoints;
    return Point(result(0), result(1));
}

RTMotionProfile::Point RTMotionProfile::Bezier::getDerivative(double t) {
    Eigen::RowVector3d derivativeOfCubicPn;
    derivativeOfCubicPn << t*t, t, 1;
    auto result = (derivativeOfCubicPn * this->derivative_coefficients) * this->controlPoints;
    return Point(result(0), result(1));
}

RTMotionProfile::Point RTMotionProfile::Bezier::getSecondDerivative(double t) {
    Eigen::RowVector2d secondDerivativeCubicPn;
    secondDerivativeCubicPn << t, 1;
    auto result = (secondDerivativeCubicPn * this->second_derivative_coefficients) * this->controlPoints;
    return Point(result(0), result(1));
}

double RTMotionProfile::Bezier::getLength() {
    return this->length;
}

double RTMotionProfile::Bezier::getCurvature(double t) {
    Point firstDerivative = this->getDerivative(t);
    Point secondDerivative = this->getSecondDerivative(t);
    double curvature = (firstDerivative.x*secondDerivative.y - firstDerivative.y*secondDerivative.x) / 
        std::pow(firstDerivative.x * firstDerivative.x + firstDerivative.y * firstDerivative.y, 1.5);
    return curvature;
}

double RTMotionProfile::Bezier::getCurvature(Point firstDerivative, Point secondDerivative) {
    double denom = firstDerivative.x*firstDerivative.x+firstDerivative.y*firstDerivative.y;
    denom *= denom*denom;
    denom = std::sqrt(denom);
    double curvature = (firstDerivative.x*secondDerivative.y-firstDerivative.y*secondDerivative.x) / denom;
    return curvature;
}

RTMotionProfile::ProfilePoint::ProfilePoint(double x, double y, double theta, double curvature, 
    double t, double vel, double accel) :
x(x), y(y), theta(theta), curvature(curvature), t(t), vel(vel), accel(accel) {}
RTMotionProfile::ProfilePoint::ProfilePoint(double dist, double vel) {
    this->dist = dist;
    this->vel = vel;
}

RTMotionProfile::Constraints::Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width) {
    this->max_vel = max_vel;
    this->max_acc = max_acc;
    this->friction_coef = friction_coef;
    this->max_dec = max_dec;
    this->max_jerk = max_jerk;
    this->track_width = track_width;
}

double RTMotionProfile::Constraints::maxSpeed(double curvature) {
    const double GRAV_COEFFICIENT_MS_2 = 9.81;
    const double METER_CONVERSION = 39.3701;
    double max_turn_speed = ((2*this->max_vel / this->track_width) * this->max_vel) / (fabs(curvature) * this->max_vel + (2*this->max_vel/this->track_width));
    if (curvature == 0) return max_turn_speed;
    double max_slip_speed = sqrt(this->friction_coef * (1/fabs(curvature))*GRAV_COEFFICIENT_MS_2*METER_CONVERSION);
    return std::min(max_slip_speed, max_turn_speed);
}

std::pair<double, double> RTMotionProfile::Constraints::wheelSpeeds(double angularVel, double vel) {
    double left_vel = vel - angularVel * this->track_width/2;
    double right_vel = vel + angularVel * this->track_width/2;
    return {left_vel, right_vel};
}

RTMotionProfile::TrapezoidalProfile::TrapezoidalProfile(std::shared_ptr<Constraints> constraints, double length, double start_vel, double end_vel) {
    this->constraints = constraints;
    this->length = length;
    this->start_vel = start_vel;
    this->end_vel = end_vel;

    double non_cruise_distance = constraints->max_vel * constraints->max_vel / (2*constraints->max_acc) + constraints->max_vel * constraints->max_vel / (2*constraints->max_dec);
    this->cruise_vel = non_cruise_distance < length ? constraints->max_vel : std::sqrt(2*(length*constraints->max_acc*constraints->max_dec)/(constraints->max_acc+constraints->max_dec));

    this->accel_dist = (this->cruise_vel*this->cruise_vel-this->start_vel*this->start_vel) / (2*constraints->max_acc);
    this->decel_dist = length + (this->end_vel * this->end_vel - this->cruise_vel*this->cruise_vel) / (2*constraints->max_dec);
}

double RTMotionProfile::TrapezoidalProfile::get_vel_at_dist(double dist) {
    if (dist < 0) dist = 0;
    if (dist > this->length) dist = this->length;

    if (dist < this->accel_dist) return std::sqrt(this->start_vel*this->start_vel+2*this->constraints->max_acc*dist);
    else if (dist < this->decel_dist) return this->cruise_vel;
    else return std::sqrt(this->cruise_vel*this->cruise_vel+2*this->constraints->max_dec*(dist-this->decel_dist));
}

RTMotionProfile::ProfileGenerator::ProfileGenerator(std::shared_ptr<Constraints> constraints, double dd) {
    this->constraints = constraints;
    this->dd = dd;
}

void RTMotionProfile::ProfileGenerator::generateProfile(std::shared_ptr<abstractPath> path) {
    this->profile.clear();
    double dist = this->dd;
    double vel = 0.0001;
    MotionProfile forwardPass;
    forwardPass.push_back(ProfilePoint(0, 0));
    double t = 0;
    double curvature;
    double angular_vel;
    double angular_accel;
    double last_angular_vel;
    double max_accel;
    Point firstDerivative;
    Point secondDerivative;

    std::vector<double> cache;

    while (t <= 1) {
        firstDerivative = path->getDerivative(t);
        secondDerivative = path->getSecondDerivative(t);

        t += dd / sqrt(firstDerivative.x * firstDerivative.x + firstDerivative.y * firstDerivative.y);
        curvature = path->getCurvature(firstDerivative, secondDerivative);
        cache.push_back(curvature);
        angular_vel = vel*curvature;
        angular_accel = (angular_vel - last_angular_vel) * (vel / dd);
        last_angular_vel = angular_vel;

        max_accel = this->constraints->max_acc - std::abs(angular_accel * this->constraints->track_width/2);
        vel = std::min(this->constraints->maxSpeed(curvature), std::sqrt(vel*vel+2*max_accel*dd));
        dist+=dd;
        forwardPass.push_back(ProfilePoint(dist, vel));
    }

    vel = 0.00001;
    last_angular_vel = 0;
    angular_accel = 0;
    t = 1;
    int index = 0;

    while (dist >= 0) {
        curvature = cache.back();
        cache.pop_back();
        angular_vel = vel*curvature;
        angular_accel = (angular_vel - last_angular_vel) * (vel / dd);
        last_angular_vel = angular_vel;

        max_accel = this->constraints->max_dec - std::abs(angular_accel*this->constraints->track_width/2);
        vel = std::min(this->constraints->maxSpeed(curvature), std::sqrt(vel*vel+2*max_accel*dd));
        dist -= dd;
        
        this->profile.push_back((ProfilePoint(
            forwardPass[index].dist,
            std::min(forwardPass[index].vel, vel)
        )));
        index++;
    }
    cache.clear();
}

RTMotionProfile::ChassisSpeeds RTMotionProfile::ProfileGenerator::getProfilePoint(double d) {
    int index = int(d/this->dd);
    if (index >= this->profile.size()) index = this->profile.size()-1;
    double vel = this->profile[index].vel;
    double curvature = this->profile[index].curvature;
    double angular_vel = vel*curvature;
    double accel = this->profile[index].accel;
    return ChassisSpeeds(vel, angular_vel, accel, Pose(
        this->profile[index].x, 
        this->profile[index].y,
        this->profile[index].theta));
}

double RTMotionProfile::ProfileGenerator::get_delta_d() { return this->dd; }

void RTMotionProfile::ProfileGenerator::followProfile(std::shared_ptr<lemlib::Chassis> _chassis) {
    double totalDistance = this->getProfile().back().dist;
    double delta_d = this->get_delta_d();
    for (double d = 0; d <= totalDistance; d+=delta_d) {
        auto wheelSpeeds = this->getProfilePoint(d);
        
        _chassis->get_drivetrain().leftMotors->move(wheelSpeeds.vel-wheelSpeeds.omega);
        _chassis->get_drivetrain().rightMotors->move(wheelSpeeds.vel+wheelSpeeds.omega);
        pros::delay(this->get_delta_d());
    }  
    _chassis->get_drivetrain().leftMotors->move(0);
    _chassis->get_drivetrain().rightMotors->move(0);
}