#include "robot.hpp"

namespace ai{
	namespace model{
		robot::robot():id_(0), x_(0), y_(0), theta_(0){}

		robot::robot(uint32_t _id, double _x, double _y):id_(_id), x_(_x), y_(_y), theta_(0){}

		uint32_t robot::id(){
			return id_;
		}

		double robot::x(){
			return x_;
		}

		double robot::y(){
			return y_;
		}

		double robot::vx(){
			return vx_;
		}

		double robot::vy(){
			return vy_;
		}

		double robot::ax(){
			return ax_;
		}

		double robot::ay(){
			return ay_;
		}

		double robot::theta(){
			return theta_;
		}

		double robot::omega(){
			return omega_;
		}

		void robot::x(double _x){
			x_ = _x;
		}

		void robot::y(double _y){
			y_ = _y;
		}

		void robot::vx(double _vx){
			vx_ = _vx;
		}

		void robot::vy(double _vy){
			vy_ = _vy;
		}

		void robot::ax(double _ax){
			ax_ = _ax;
		}

		void robot::ay(double _ay){
			ay_ = _ay;
		}

		void robot::theta(double _theta){
			theta_ = _theta;
		}

		void robot::omega(double _omega){
			omega_ = _omega;
		}
	}
}
