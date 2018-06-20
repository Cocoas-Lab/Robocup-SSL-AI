#ifndef AI_MODEL_ROBOT_HPP_
#define AI_MODEL_ROBOT_HPP_

#include <stdint.h>

namespace ai{
	namespace model{
		class robot{
			uint32_t id_;
			double x_;
			double y_;
			double vx_;
			double vy_;
			double ax_;
			double ay_;
			double theta_;
			double omega_;

		public:
			robot();
			robot(uint32_t _id);
			robot(uint32_t _id, double _x, double _y);
			robot(const robot&) = default;

			uint32_t id();
			double x();
			double y();
			double vx();
			double vy();
			double ax();
			double ay();
			double theta();
			double omega();

			void x(double _x);
			void y(double _y);
			void vx(double _vx);
			void vy(double _vy);
			void ax(double _ax);
			void ay(double _ay);
			void theta(double _theta);
			void omega(double _omega);
		};
	}
}

#endif
