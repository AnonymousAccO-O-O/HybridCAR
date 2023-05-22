/*
	Copyright (C) 2018, Jianwen Li (lijwen2748@gmail.com), Iowa State University

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef NEW_MAIN_SOLVER_H
#define NEW_MAIN_SOLVER_H

#include "carsolver.h"
#include "data_structure.h"
#include "model.h"
#include "statistics.h"
#include <vector>
#include <assert.h>
#include <iostream>

namespace car
{
	extern Statistics stats;

	class MainSolver : public CARSolver
	{
	public:
		MainSolver(Model *, const bool verbose = false);
		~MainSolver() {}

		void set_assumption(const Assignment &, const int);

		bool solve_with_assumption();
		bool solve_with_assumption(const Assignment &st, const int p);

		State* get_state(const bool forward);
		Assignment get_state_full_assignment(const bool forward);

		// this version is used for bad check only
		Cube get_conflict(const int bad);
		Cube get_conflict(const bool forward, bool&constraint);

		// void add_new_frame(const Frame &frame, const int frame_level, const bool forward);

		inline void update_constraint(Cube &cu)
		{
			CARSolver::add_clause_from_cube(cu);
		}

		void shrink_model(Assignment &model);

	public:
	// This section is for bi-car

		// silly method: clear the main solver when seraching with the O sequence ends.
		// clever method: record flag of different levels in each O sequence.
		static int bi_max_flag;
		static hash_map<Osequence*,std::vector<int>> flag_of_O;

		inline int bi_flag_of(Osequence *o, const int frame_level)
		{
			assert(frame_level >= 0);
			while(frame_level >= flag_of_O[o].size())
			{
				flag_of_O[o].push_back(bi_max_flag++);
			}
			return flag_of_O[o][frame_level];
		}
		
		void bi_record_O(Osequence *o, bool dir)
		{
			// should not have met before.
			assert(flag_of_O.count(o) == 0); 
			flag_of_O[o] = {};
		}

		// void bi_set_assumption_disposable(Osequence *O, State *s, const int frame_level, const bool forward, const std::vector<Cube>& prefers);

		// void bi_dispose();

		/**
		 * @brief set assumption = { s->s() , flag_of(Os[frame_level]) }
		 * 
		 * @param s 
		 * @param frame_level 
		 * @param forward 
		 */
		void bi_set_assumption(Osequence *O, State *s, const int frame_level, const bool forward);

		void bi_set_assumption(Osequence *O, State *s, const int frame_level, const bool forward, const std::vector<Cube>& prefers);
		
		void bi_add_new_frame(const Frame& frame, const int frame_level, Osequence *O, const bool forward);

		void bi_add_clause_from_cube(const Cube &cu, const int frame_level, Osequence *O, const bool forward);


	private:
		Model *model_;
	};

}

#endif
