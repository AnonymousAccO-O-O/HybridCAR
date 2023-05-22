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
 
#ifndef CAR_SOLVER_H
#define	CAR_SOLVER_H

#ifdef MINISAT
#include "minisat/core/Solver.h"
#else
#include "glucose/core/Solver.h"
#endif // DEBUG
// #include "minisat/core/Solver.h"
// #include "glucose/core/Solver.h"
#include "statistics.h"
#include <vector>

namespace car
{
	extern Statistics stats;
	class CARSolver : 
	#ifdef MINISAT
	public Minisat::Solver
	#else
	public Glucose::Solver 
	#endif // DEBUG
	{
	public:
		CARSolver () {verbose_ = false;}
		CARSolver (bool verbose) : verbose_ (verbose) {} 
		bool verbose_;

		#ifdef MINISAT
		Minisat::vec<Minisat::Lit> assumption_;  //Assumption for SAT solver
		#else
		Glucose::vec<Glucose::Lit> assumption_;  //Assumption for SAT solver
		#endif // DEBUG
		// Minisat::vec<Minisat::Lit> assumption_;  //Assumption for SAT solver
		// Glucose::vec<Glucose::Lit> assumption_;  //Assumption for SAT solver
		
//functions
		bool solve_assumption ();
		std::vector<int> get_model () const;    //get the model from SAT solver
 		std::vector<int> get_uc () const;       //get UC from SAT solver
		std::vector<int> get_uc_no_bad (int bad) const; //get UC from SAT solver
		
		void add_cube (const std::vector<int>&);
		void add_clause_from_cube (const std::vector<int>&);
		void add_clause (int);
 		void add_clause (int, int);
 		void add_clause (int, int, int);
 		void add_clause (int, int, int, int);
 		void add_clause (std::vector<int>&);
 	
		#ifdef MINISAT
		Minisat::Lit SAT_lit (int id); //create the Lit used in SAT solver for the id.
		int lit_id (Minisat::Lit) const;  //return the id of SAT lit
		#else
		Glucose::Lit SAT_lit (int id); //create the Lit used in SAT solver for the id.
 		int lit_id (Glucose::Lit) const;  //return the id of SAT lit
		#endif // DEBUG
		
 		
 		

 		
 		inline int size () {return clauses.size ();}
 		inline void clear_assumption () {assumption_.clear ();}
 		inline std::vector<int> get_assumption()
		{
			std::vector<int> res;
			res.reserve(assumption_.size());
			for(int i = 0 ; i < assumption_.size();++i)
			{
				res.push_back(lit_id(assumption_[i]));
			}
			return res;
		}
 		
 		//printers
		void print_last_3_clauses();
 		void print_clauses ();
 		void print_assumption ();
 		
 		//l <-> r
 		inline void add_equivalence (int l, int r)
 		{
 			add_clause (-l, r);
 			add_clause (l, -r);
 		}
 	
 		//l <-> r1 /\ r2
 		inline void add_equivalence (int l, int r1, int r2)
 		{
 			add_clause (-l, r1);
 			add_clause (-l, r2);
 			add_clause (l, -r1, -r2);
 		}
 	
 		//l<-> r1 /\ r2 /\ r3
 		inline void add_equivalence (int l, int r1, int r2, int r3)
 		{
 			add_clause (-l, r1);
 			add_clause (-l, r2);
 			add_clause (-l, r3);
 			add_clause (l, -r1, -r2, -r3);
 		}

		inline void setReduceBy(double x)
		{
			reduceBy = x;
		}
		
		inline double getReduceBy() const
		{
			return reduceBy;
		}
	};
}

#endif
