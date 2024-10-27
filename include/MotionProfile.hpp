#pragma once

namespace Profile {
    struct Constraints {

    };
    struct Point {
        private:
            double x;
            double y;
        public: 
        virtual double getDistance(Point point);
        Point() { 
            x = 0; 
            y = 0;
        };
        Point(double x, double y) :
            x(x),
            y(y) {}

    };
    struct Pose : public Point {
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
            virtual double tAtArcLength() = 0;
            virtual double getCurvature(double t) = 0;
            virtual double getCurvature(Point p1, Point p2) = 0;
    };
    class Bezier : public abstractPath {
        public:
            double getLength();
            Point getPoint(double t);
            Point getDerivative(double t);
            Point getSecondDerivative(double t);
            double tAtArcLength();
            double getCurvature(double t);
            double getCurvature(Point p1, Point p2);
    };
}