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

#include "newmainsolver.h"
#include "utility.h"
#include "DEBUG_PRINTING.h"
#include <algorithm>
using namespace std;
namespace car
{
	int MainSolver::bi_max_flag = -1;
	hash_map<Osequence*,std::vector<int>> MainSolver::flag_of_O;

	/**
	 * @brief 把constraints、outputs和latches加入clause。
	 * 		
	 * @param m 
	 * @param verbose 
	 */
	MainSolver::MainSolver (Model* m, const bool verbose) 
	{
	    verbose_ = verbose;
		model_ = m;
		if (bi_max_flag == -1)
			bi_max_flag = m->max_id() + 1;
	    //constraints
		# ifdef PRINT_MAINSOVLER_INIT
		cout<<"[ Init Main Solver ]:"<<endl;
		# endif		
		for (int i = 0; i < m->outputs_start (); i ++)
		{
			add_clause (m->element (i));
			# ifdef PRINT_MAINSOVLER_INIT
			cout<<"Latch equivalence or initialization: ";
			for(auto l: m->element(i))
				cout<<l<<", ";
			cout<<endl;
			# endif	
		}
		//outputs
		for (int i = m->outputs_start (); i < m->latches_start (); i ++)
		{
			add_clause (m->element (i));
			# ifdef PRINT_MAINSOVLER_INIT
			cout<<"Equivalance by AndGates: ";
			for(auto l: m->element(i))
				cout<<l<<", ";
			cout<<endl;
			# endif	
		}
		//latches
		for (int i = m->latches_start (); i < m->size (); i ++)
		{
			add_clause (m->element (i));
			# ifdef PRINT_MAINSOVLER_INIT
			assert(m->size() >= 2);
			if(i < m->size()-2)
			{
				cout<<"Latches: ";
				for(auto l: m->element(i))
					cout<<l<<", ";
				cout<<endl;
			}
			else if(i == m->size() -2)
			{
				cout<<"True value: ";
				for(auto l: m->element(i))
					cout<<l<<", ";
				cout<<endl;
			}
			else if(i == m->size() -1)
			{
				cout<<"False value: ";
				for(auto l: m->element(i))
					cout<<l<<", ";
				cout<<endl;
			}
			else cout<<"Error happens in printing"<<endl;	
			# endif	
		}
		# ifdef PRINT_MAINSOVLER_INIT
		print_assumption();
		cout<<endl<<"[ End of Init Main Solver] "<<endl<<endl;
		# endif	
	}
	
	/**
	 * @brief assumption_ = {assum, bad_id}
	 * 
	 * @param assum 
	 * @param bad_id 
	 */
	void MainSolver::set_assumption (const Assignment& assum, const int bad_id)
	{
		assumption_.clear ();
		for (const int &var :assum)
		{
			assumption_.push (SAT_lit (var));
		}
		assumption_.push (SAT_lit (bad_id));		
	}

	//NOTE: this State* is not owned by this solver, but the checker. It should be immediately added into clear duty.	
	State* MainSolver::get_state (const bool forward)
	{
		Assignment model = get_model();
		if(!forward)
			shrink_model (model);
		Assignment inputs(model.begin(),model.begin() + model_->num_inputs());
		Assignment latches(model.begin() + model_->num_inputs(),model.begin() + model_->num_inputs()+model_->num_latches());
		State* s = new State(inputs,latches);
		return s;
	}

	Assignment MainSolver::get_state_full_assignment (const bool forward)
	{
		Assignment model = get_model();
		if(!forward)
			shrink_model (model);
		return model;
	}
	
	//this version is used for bad check only
	/**
	 * @brief get unsat core, and remove bad from it if exists.
	 * 
	 * @param bad 
	 * @return Cube 
	 */
	Cube MainSolver::get_conflict (const int bad)
	{
		Cube res = get_uc_no_bad (bad);
		if(res.empty())
			return res;
		int last_lit = res.back();
		sort(res.begin(),res.end(),[](int&a, int&b){return abs(a) < abs(b);});
		res.push_back(last_lit);
		return res;
	}
	
	/**
	 * @brief get unsat core, get latch, shrink to previous if forward. 
	 * yes, it is never used.
	 * to mark whether uc is empty, therefore T is unsatisfiable.
	 * @param forward 
	 * @param constraint 
	 * @return Cube 
	 */
	Cube MainSolver::get_conflict (const bool forward, bool&constraint)
	{
		Cube conflict = get_uc ();
		assert(!conflict.empty());
		// cout<<"conflict get from solver: ";
		// for(int i:conflict)
		// cout<<i<<", ";
		// cout<<endl;
		if (forward)
		{
			// TODO: add constraint checking here.
		    model_->shrink_to_previous_vars (conflict, constraint);
		}
		else
		{
			// FIXED: real global uc?
		    model_->shrink_to_latch_vars (conflict, constraint);
		}
		// cout<<"conflict after shrink: ";
		// for(int i:conflict)
		// cout<<i<<", ";
		// cout<<endl;
		int conflict_back = conflict.back();
		std::sort (conflict.begin (), conflict.end (), car::comp);
		conflict.push_back(conflict_back);
		return conflict;
	}
	
	void MainSolver::bi_add_new_frame(const Frame& frame, const int frame_level,Osequence *O, const bool forward)
	{
		for (int i = 0; i < frame.size (); i ++)
		{
			bi_add_clause_from_cube (frame[i], frame_level, O, forward);
		}
	}

	// Actually, each center_state should only exist in O sequence in one direction.
	// Should we just record this?
	void MainSolver::bi_add_clause_from_cube(const Cube &cu, const int frame_level, Osequence *O, const bool forward)
	{
		int flag = bi_flag_of (O,frame_level);
		vector<int> cl = {-flag};
		for (int i = 0; i < cu.size (); i ++)
		{
			if (!forward)
				cl.push_back (-model_->prime (cu[i]));
			else
				cl.push_back (-cu[i]);
		}
		add_clause (cl);
	}

    void MainSolver::bi_set_assumption(Osequence* O, State*s,  const int frame_level, const bool forward)
    {
		assumption_.clear();
		if (frame_level > -1)
			assumption_.push (SAT_lit (bi_flag_of(O,frame_level)));
		for (const int &id :s->s())
		{
			int target = forward ? model_->prime (id) : id;
			assumption_.push (SAT_lit (target));
		}
		# ifdef PRINT_ASSUMPTION
		cout<<"The assumption is :";
		for(int i = 0; i < assumption_.size() ; ++i)
		{
			cout<<lit_id(assumption_[i])<<" ";
		}
		cout<<endl<<endl;
		# endif // PRINT_ASSUMPTION
    }

	void MainSolver::bi_set_assumption(Osequence* O, State*s,  const int frame_level, const bool forward, const std::vector<Cube> & prefers)
    {
		assumption_.clear();
		if (frame_level > -1)
			assumption_.push (SAT_lit (bi_flag_of(O,frame_level)));
		

		for(int i = 0; i < prefers.size(); ++i)
		{
			auto &vec = prefers[i];
			// for intersections, place the last lit in the front may get smaller uc?

			for(auto &id : vec)
			{
				int target = forward?model_->prime(id) : id;
				assumption_.push(SAT_lit(target));
			}
		}
		for (const int &id :s->s())
		{
			int target = forward ? model_->prime (id) : id;
			assumption_.push (SAT_lit (target));
		}
		# ifdef PRINT_ASSUMPTION
		cout<<"The assumption is :";
		for(int i = 0; i < assumption_.size() ; ++i)
		{
			cout<<lit_id(assumption_[i])<<" ";
		}
		cout<<endl<<endl;
		# endif // PRINT_ASSUMPTION
    }

	// FIXME: rewrite here.
    void MainSolver::shrink_model (Assignment& model)
	{
	    Assignment res=model;
		for (int i = model_->num_inputs ()+1; i <= model_->num_inputs () + model_->num_latches (); i ++)
		{
			int p = model_->prime (i);
			assert (p != 0);
			assert (model.size () > abs (p));
			
			int val = model[abs(p)-1];
			if (p == val)
				res[i-1]=(i);
			else
				res[i-1]=(-i);
		}
		model = res;
	}
	

	bool MainSolver::solve_with_assumption (const Assignment& st, const int p)
	{
		stats.count_main_solver_SAT_time_start();
		set_assumption (st, p);
		if (verbose_)
			std::cout << "MainSolver::";
		bool res = solve_assumption ();
		stats.count_main_solver_SAT_time_end();
		return res;
	}

	bool MainSolver::solve_with_assumption ()
	{
		stats.count_main_solver_SAT_time_start();
		if (verbose_)
			std::cout << "MainSolver::";
		bool res = solve_assumption ();
		stats.count_main_solver_SAT_time_end();
		return res;
	}

	
	
}
