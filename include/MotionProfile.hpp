#pragma once
#include <vector>
#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
namespace RTMotionProfile {
    struct Point {
        public:
            double x;
            double y;
            double getDistance(Point point);
            Point() { 
                x = 0; 
                y = 0;
            }
            Point(double x, double y) :
                x(x),
                y(y) 
            {}

    };
    struct Pose {
        private:
            double x;
            double y;
            double theta;
        public: 
            double getDistance(Pose pose);
            double getHeading(Pose pose);
            Pose(double x, double y, double theta){
                this->x = x;
                this->y = y;
                this->theta = theta;
            }
            Pose() {
                x = 0;
                y = 0;
                theta = 0;
            }
    };
    class abstractPath {
        public:
            virtual double getLength() = 0;
            virtual Point getPoint(double t) = 0;
            virtual Point getDerivative(double t) = 0;
            virtual Point getSecondDerivative(double t) = 0;
            virtual double tAtArcLength(double arcLength) = 0;
            virtual double getCurvature(double t) = 0;
            virtual double getCurvature(Point firstDerivative, Point secondDerivative) = 0;
    };
    class Bezier : public abstractPath {
        public:
            double getLength();
            Point getPoint(double t);
            Point getDerivative(double t);
            Point getSecondDerivative(double t);
            double tAtArcLength(double arcLength);
            double getCurvature(double t);
            double getCurvature(Point firstDerivative, Point secondDerivative);

        Bezier(const Point &p0, const Point &p1, const Point &p2, const Point &p3, int length = 10000);
        ~Bezier() {
            this->lengths.clear();
        }
        private:
            Point A;
            Point B;
            Point C;
            Point D;
            Eigen::Matrix<double, 4, 2> controlPoints;
            std::vector<double> lengths;
            Eigen::Matrix<double, 4, 4> coefficients;
            Eigen::Matrix<double, 3, 4> derivative_coefficients;
            Eigen::Matrix<double, 2, 4> second_derivative_coefficients;
            double length;
            double len;
            double curvature;
            int length_samples;
    };
    struct Constraints {
        Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width);
        double maxSpeed(double curvature);
        std::pair<double, double> wheelSpeeds(double angularVel, double vel);
        double max_vel;
        double max_acc;
        double friction_coef;
        double max_dec;
        double max_jerk;
        double track_width;
    };
    class ProfilePoint {
    public:
        ProfilePoint(double x, double y, double theta, double curvature, double t, double vel, double accel);
        ProfilePoint(double dist, double vel);
        double x;
        double y;
        double theta;
        double curvature;
        double t;
        double vel;
        double accel;
        double dist;
    };
    typedef std::vector<ProfilePoint> MotionProfile;
    class ChassisSpeeds {
    public:
        ChassisSpeeds(double vel, double omega, double accel, Pose pose)
        {
            this->vel = vel;
            this->omega = omega;
            this->accel = accel;
            this->pose = pose;
        }
        double vel;
        double omega;
        double accel;
        Pose pose;
    };
    class ProfileGenerator {
    public:
        ProfileGenerator(Constraints *constraints, double dd);
        void generateProfile(abstractPath *path);
        ChassisSpeeds getProfilePoint(double d);
        auto getProfile() { return profile; }

    private:
        Constraints *constraints;
        MotionProfile profile;
        double dd;
        double duration;
    };

    class TrapezoidalProfile
    {
    public:
        TrapezoidalProfile(Constraints *constraints, double length, double start_vel = 0, double end_vel = 0);
        double get_vel_at_dist(double dist);

    private:
        Constraints *constraints;
        double length;
        double start_vel;
        double end_vel;
        double cruise_vel;
        double accel_dist;
        double decel_dist;
    };
}