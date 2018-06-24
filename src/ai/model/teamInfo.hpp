#ifndef AI_MODEL_TEAM_INFO_HPP_
#define AI_MODEL_TEAM_INFO_HPP_

#include <string>
#include <stdint.h>

namespace ai{
	namespace model{
		class teamInfo{
			std::string name_;
			uint32_t score_;
			uint32_t goalie_;
			uint32_t redCards_;
			uint32_t yellowCards_;
			uint32_t yellowCardTimes_;
			uint32_t timeouts_;
			uint32_t timeoutTimes_;

			public:
			teamInfo() = delete;
			explicit teamInfo(const std::string& _name);

			const std::string& name()const;
			uint32_t score()const;
			uint32_t goalie()const;
			uint32_t redCards()const;
			uint32_t yellowCards()const;
			uint32_t yellowCardTimes()const;
			uint32_t timeouts()const;
			uint32_t timeoutTimes()const;

			void score(uint32_t _score);
			void goalie(uint32_t _goalie);
			void redCards(uint32_t _redCards);
			void yellowCards(uint32_t _yellowCards);
			void yellowCardTimes(uint32_t _yellowCardTimes);
			void timeouts(uint32_t _timeouts);
			void timeoutTimes(uint32_t _timeoutTimes);
		};
	}
}

#endif
