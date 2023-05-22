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

#include "newchecker.h"
#include "newpartialsolver.h"
#include <vector>
#include <iostream>
#include <stack>
#include <functional>
#include "utility.h"
#include "statistics.h"
#include <algorithm>
#include "DEBUG_PRINTING.h"
#include <unordered_set>
#include <queue>
#include <tuple>
#include <random>
#include <chrono>
using namespace std;
using namespace std::chrono;
using clock_high = time_point<high_resolution_clock>;



namespace car
{
	// increase one each time. monotonous
	int storage_id = 0;
	static vector<Cube> reorder_assumption(vector<Cube> inter, const Cube& rres, const Cube& rtmp, const Cube& sml, const Cube& s_next, const Cube& pres_next, const Cube & recent, const Cube & more_inter);
	static Cube intersect(const Cube& c1, const Cube& c2);
	static Cube minus(const Cube& c1, const Cube& c2);


	void Checker::print_U_sequnece(const Usequence & U, ostream & out_stream)
	{
		int B_level = 0;
		out_stream << "------ Start Printing U sequence ------" << endl;
		for (auto frame : U)
		{
			out_stream << "U level : " << B_level++ << endl;
			for (auto state : frame)
			{
				out_stream << "(";
				for (auto lit : state->s())
				{
					out_stream << (lit > 0 ? '+' : '-');
				}
				out_stream << "), id = "<<state->id << endl;
			}
			out_stream << endl;
		}
		out_stream << "------ End Printing U sequence ------" << endl << endl;
	}

	int Checker::reorder_intersect_one(Cube &asgn, const State *s, int max_index)
	{
		if (!s)
			return max_index;
		Cube latches = s->s();
		int offset = model_->num_inputs() + 1;

		int forward_index = 0;
		Cube tmp;
		for (int i = 0; i <= max_index; ++i)
		{
			int index = abs(asgn[i]) - offset;
			if (latches[index] == asgn[i])
				asgn[forward_index++] = asgn[i];
			else
				tmp.push_back(asgn[i]);
		}
		for (int i = 0; i < tmp.size(); ++i)
		{
			asgn[forward_index + i] = tmp[i];
		}

		return forward_index - 1;
	}

	void Checker::reorder_intersect_unittest(State*s)
	{
		// like qsort, but in wrong order
		// int forward_index = 0;
		// int backward_index = max_index;
		// while(forward_index <= backward_index)
		// {
		// 	int index = abs(asgn[forward_index])-offset;
		// 	// same sign
		// 	if(latches[index] == asgn[forward_index])
		// 	{
		// 		++forward_index;
		// 	}
		// 	else{
		// 		int tmp = asgn[forward_index];
		// 		asgn[forward_index] = asgn[backward_index];
		// 		asgn[backward_index] = tmp;
		// 		--backward_index;
		// 	}
		// }


		if(which_U().size() > 1)
		{
			auto &Uframe =  which_U()[which_U().size()-1];
			if(Uframe.size() >= 1)
			{
				auto last_state = Uframe[Uframe.size() - 1];
				Cube state = s->s();
				cerr<<"max index : "<<state.size()-1<<endl;
				cerr<<"asgn : ";
				for(int i:state)
				cerr<<i<<", ";
				cerr<<endl;
				cerr<<"state : ";
				for(int i:last_state->s())
				cerr<<i<<", ";
				cerr<<endl;

				int pos = reorder_intersect_one(state,last_state, state.size()-1);


				cerr<<"after : ";
				for(int i:state)
				cerr<<i<<", ";
				cerr<<endl;

				cerr<<"return: "<<pos<<endl;

				// it should be a fixpoint
				Cube tmp_state = state;
				int qos = reorder_intersect_one(state,last_state,pos);
				assert(pos == qos);
				for(int i = 0 ;i <state.size();++i)
				{
					assert(state[i] == tmp_state[i]);
				}

			}
		}
	}

	void Checker::reorder_intersect(Cube& asgn, const std::vector<State*>& frame, std::vector<int>& pos)
	{
		int max_index = asgn.size()-1;
		int cnt = 1;
		while(max_index >0 && cnt+1 <= frame.size())
		{
			State* s = frame[frame.size()-cnt-1];
			max_index = reorder_intersect_one(asgn,s,max_index);
			if(max_index <0)
				return; 
			pos.push_back(max_index);
			cnt++;
		}
		return;
	}

	int Checker::reorder_intersect_how_many(const Cube& asgn,const Cube& uc,std::vector<int>& pos)
	{
		unordered_map<int,int> helper;
		for(int i = 0 ; i < asgn.size();++i)
		{
			helper[asgn[i]] = i;
		}
		int max_index = -1;
		for(int i = 0 ; i < uc.size();++i)
		{
			if(!helper.count(uc[i]))
				return 0;
			max_index = max(max_index,helper[uc[i]]);
		}
		for(int i = 0; i < pos.size(); ++i)
		{
			if(pos[i] < max_index)
				return i; // -1 + 1 = 0
		}
		return pos.size();
	}

	void Checker::reorder_intersect_how_many_unittest(State *s)
	{
		if(which_U().size() > 0)
		{
			Cube counter;
			auto& frame = which_U()[which_U().size()-1];
			Cube cu = s->s();
			reorder_intersect(cu,frame,counter);

			// solver->bi_set_assumption(O,s,level,forward,{cu});
			// res = solver->solve_with_assumption();
			bool res;
			if(!res)
			{
				cerr<<"counter: ";
				for(int i = 0; i<counter.size();++i)
					cerr<<i<<" : "<<counter[i]<<", ";
				cerr<<endl;
				bool cons=true;
				Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
				int how_many = reorder_intersect_how_many(cu,uc,counter);
				cerr<<"count = "<< how_many <<endl;
			}
		}

	}

    static void print_O_sequence(const Osequence &O, const Frame& Otmp, ostream & out_stream)
	{
		int O_level = 0;
		out_stream << "------ Start Printing O sequence ------" << endl;
		for (auto frame : O)
			{
				out_stream << "O level : " << O_level++ << endl;
				for (auto cube : frame)
				{
					out_stream << "(";
					for (auto lit : cube)
					{
						out_stream << lit << ", ";
					}
					out_stream << ")," << endl;
				}
				out_stream << endl;
			}
		{
			out_stream << "O level : " << "Otmp" << endl;
			for (auto cube : Otmp)
			{
				out_stream << "(";
				for (auto lit : cube)
				{
					out_stream << lit << ", ";
				}
				out_stream << ")," << endl;
			}
			out_stream << endl;
		}

		out_stream << "------ End Printing O sequence ------" << endl;
	}

	static int count_of_states(Usequence &U)
	{
		int cnt = 0;
		for(auto& frame:U)
			cnt += frame.size();
		return cnt;
	}

	bool Checker::bi_check()
	{
		bool trivial_res;
		if(trivial_check(trivial_res))
		{
			return trivial_res ? true : false; 
		}
		bool res = car_inv();
		stats.count_maingoal_end(),stats.count_for_car_end(),stats.count_after_car_start();
		if (res)
	    	out << "0" << endl;
  		else
		    out << "1" << endl;
  		out << "b" << 0 << endl;
		
		
		if(evidence_ && !res)
		{
			bi_print_evidence();
			print_U_shape();
			if(dot_out.is_open())
			{
				bi_draw_graph();
			}
			if(dive_out.is_open())
			{
				dive_mark_bad();
      			dive_draw_tail();
			}

			// print_O_sequence(Onp,{},cout);
			// print_U_sequnece(Ub,cout);
			// print_O_sequence(Onp,{},cout);
			// print_U_sequnece(Ub,cout);
		}
		LOG("End");
		out<<"."<<endl;
		stats.count_clean_start();
		bi_clean();
		stats.count_clean_end();
		stats.count_after_car_end();
		return res;
	}


	/**
	 * @brief 
	 * <V,I,T>, P
	 * Under-Approximate: set of States
	 * Over-Approximate: 
	 * SAT(I /\ T /\ ~P) (assumption : I)
	 *  SAT -> unsafe
	 *  UNSAT ->  
	 *  I:{a0 = 1 /\ a1 = 1 /\ a2 = 1}
	 *  uc1: {a0 == 1}
	 * SAT(I /\ T /\ ~uc1)
	 *  SAT -> t
	 * SAT(t /\ T/\ ~P)
	 */
	// NOTE: because forward_CAR seems to enumerate too many start states from ~p, it is not efficient (at least with present picking strategy)
	// NOTE: this function used to be the main entrance, now we move to car_inv, and this may not be new.
    bool Checker::bi_car()
    {
		bool res = false;
		if(bi_init_sequences(res))
		{
			return res;
		}
		static int counter = 0;
		/**
		 * As to backward search,
		 * @post Ob, Ub are both initialized.
		 * uc of UNSAT(I, ~P) is inserted into O[0].
		 * O[0] is inserted into MainSolver. 
		 */
		stats.count_before_car_end(),stats.count_for_car_start();
		while (true)
		{
			bool direction = !backward_first;
			LOG((direction ? "F" : "B"));
			Usequence &U = direction ? Uf : Ub;
			stats.count_subgoal_start();
			if(enable_bi_check)
			{	
				Usequence &V = direction ? Ub : Uf;
				# ifdef PICK_GUIDE_RANDOM
				while(State *s = pick_state_random (V))
				#else
				while(State *s = pick_state(V)) // this state works as the target
				#endif
				{
					if(blocked_states.find(s)!=blocked_states.end()) continue;
					other_counter_start() = s;
					LOG("Bi-Target:"<<s->id);
					Osequence *Os = create_O_with_state(s);
					if (bi_try_satisfy(U, Os, direction,res))
					{
						if(res)
						{
							blocked_states.insert(s);
							// block in solver
							block_state_forever(s);
							// clear Os and remove s from state -> Osequence map;
							clear_Os(s,Os);
							remove_s_from_U(s,U);
							continue;
						}
						else
							return res; // unsafe if not safe-reported.
					}
					if (bi_invariant_found(Os))
					{
						// block this state
						blocked_states.insert(s);
						// block in solver
						block_state_forever(s);
						// clear Os and remove s from state -> Osequence map;
						clear_Os(s,Os);
						remove_s_from_U(s,U);
					}
				}
			}
			stats.count_subgoal_end(),stats.count_maingoal_start();
			Osequence &O = direction ? OI : Onp;
			LOG("Bi-Target:"<< (direction ? "I" : "~P"));
			if (bi_try_satisfy(U, &O, direction,res))
			{
				return res;
			}
			// FIXME: how to reuse this.
			if (bi_invariant_found(&O))
				return true;
			stats.count_maingoal_end();
			// The magic number is 5 here.
			// 4 forward and 1 backward.
			if(enable_bi_check && (counter % 5 == 0 || counter % 5 == 1))
			{
				backward_first = !backward_first;	
				++counter;
			}
		}

		// dead code. Should not reach
		return false;
	}

	bool Checker::car_inv()
	{
		bool res = false;
		if(inv_init_sequences(res))
		{
			return res;
		}
		static int counter = 0;
		/**
		 * As to backward search,
		 * @post Ob, Ub are both initialized.
		 * uc of UNSAT(I, ~P) is inserted into O[0].
		 * O[0] is inserted into MainSolver. 
		 */
		stats.count_before_car_end(),stats.count_for_car_start();
		while (true)
		{
			bool direction = !backward_first;
			LOG((direction ? "F" : "B"));
			Usequence &U = direction ? Uf : Ub;
			stats.count_subgoal_start();
			if(enable_bi_check)
			{	
				// cerr<<"enter bi check part"<<endl;
				Usequence &V = direction ? Ub : Uf;
				# ifdef PICK_GUIDE_RANDOM
				while(State *s = pick_state_random (V))
				#else
				while(State *s = pick_state(V)) // this state works as the target
				#endif
				{
					if(blocked_states.find(s)!=blocked_states.end()) continue;
					other_counter_start() = s;
					LOG("Bi-Target:"<<s->id);
					Osequence *Os = create_O_with_state(s);
					#ifdef COMMON
					if (common_try_satisfy(U, Os, direction,res))
					#else
					if (bi_try_satisfy(U, Os, direction,res))
					#endif // COMMON
					{
						if(ppstoped)
						{
							break;
						}
						if(res)
						{
							blocked_states.insert(s);
							// block in solver
							block_state_forever(s);
							// clear Os and remove s from state -> Osequence map;
							clear_Os(s,Os);
							remove_s_from_U(s,U);
							continue;
						}
						else
							return res; // unsafe if not safe-reported.
					}
					if (bi_invariant_found(Os))
					{
						// block this state
						blocked_states.insert(s);
						// block in solver
						block_state_forever(s);
						// clear Os and remove s from state -> Osequence map;
						clear_Os(s,Os);
						remove_s_from_U(s,U);
					}
				}
			}
			stats.count_subgoal_end(),stats.count_maingoal_start();
			Osequence &O = direction ? OI : Onp;
			LOG("Bi-Target:"<< (direction ? "I" : "~P"));

			#ifdef COMMON
			if (common_try_satisfy(U, &O, direction,res))
			#else
			if (bi_try_satisfy(U, &O, direction,res))
			#endif // COMMON
			{
				if(ppstoped)
				{
					pick_next_pp();
					continue;
				}
				return res;
			}
			// FIXME: how to reuse this.
			// during pp, no need to check this. Only main procedure need to check this.
			if(!isppnow)
				if (bi_invariant_found(&O))
				return true;
			stats.count_maingoal_end();

			#if defined(SIMPU_HEAVY)
			simplify_U_descendant();
			#elif defined(SIMPU_LIGHT)
			simplify_U_children();
			#endif
			if(enable_bi_check && (counter % 5 == 0 || counter % 5 == 1))
			{
				backward_first = !backward_first;	
				++counter;
			}
		}

		// dead code. Should not reach
		return false;
	}

	bool Checker::trivial_check(bool& res)
	{
		const Model* model = model_;
		// FIXME: fix for multiple properties. 
		int bad = bad_;
		if (bad == model->true_id())
		{
			out << "1" << endl;
			out << "b" << "0" << endl;
			if (evidence_)
			{
				// print init state
				// FIXME: fix for latch inits.
				for(int i = 0; i < model->num_latches(); ++i)
					out<< "0";
				out<<endl;
				// print an arbitary input vector
				for (int j = 0; j < model->num_inputs(); j++)
					out << "0";
				out << endl;
			}
			out << "." << endl;
			if (verbose_)
			{
				cout << "return SAT since the output is true" << endl;
			}
			res = true;
			return true;
		}
		else if (bad == model->false_id())
		{
			out << "0" << endl;
			out << "b" << endl;
			out << "." << endl;
			if (verbose_)
			{
				cout << "return UNSAT since the output is false" << endl;
			}
			res = false;
			return true;
		}
		return false;		
	}

	using item = std::tuple<car::State *, int, std::size_t>;
	struct CompareItem {
		bool operator()(const item &a, const item &b) const {
			return int(std::get<2>(a)) >= int(std::get<2>(b)); // 使用元组的第三个成员进行比较
		}
	};

	void Checker::pick_next_pp()
	{
		// ##############  restore section  ##############
		cout<<"enter pp pick at : "<<duration_cast<milliseconds>(high_resolution_clock::now() - stats.global_start).count()/1000 <<endl;
		clock_high enter = high_resolution_clock::now();
		delete(bi_main_solver);
		// MainSolver::flag_of_O.clear();
		// MainSolver::bi_max_flag= -1;
		// State::next_id_ = 0; 
		bi_main_solver = new MainSolver(model);
		if(!backward_first)
		{
			delete(bi_start_solver);
			bi_start_solver = new StartSolver(model,bad_,true);
		}
		which_prior().clear();
		
		// may not need to be cleared, but for memory saving, we do it here.
		for(int i = 0; i< which_U().size();++i)
		{
			auto frame = which_U()[i];
			for(auto s: frame)
			{
				delete s;
				clear_duties.erase(s);
			}
		}
		// only remain the init size
		Ub.resize(0);
		Uf.resize(0);
		Uset.clear();
		fresh_levels.clear();

		if(backward_first)
		{
			uc_knowledge.push_back(Onp);
		}
		else
		{
			uc_knowledge.push_back(OI);
		}
		Onp.clear();
		OI.clear();
		SO_map.clear();
		rotates.clear();
		rotate.clear();
		bool res;
		inv_init_sequences(res);

		// ##############  pick section  ##############
		cout<<"next pp picked"<<endl;

		int maxni=6;
		#ifdef MAXNI
		maxni=MAXNI;
		#endif // MAXNI
		int ni = get_inter_cnt();
		bool nr =get_rotate();
		
		// // for intel042 bug
		// isppnow=false;
		// ppstoped=false;
		// set_inter_cnt(4);
		// set_rotate(false);
		// cout<<"I:"<<get_inter_cnt()<<", R:"<<get_rotate()<<endl;
		// return;

		if(ni == 0 && !nr)
		{
			set_inter_cnt(0);
			set_rotate(true);
		}
		// try all rotates
		else if(nr)
		{
			if(ni == maxni)
			{
				// reset to no rotate and do next round.
				set_inter_cnt(1);
				set_rotate(false);
			}
			else
				set_inter_cnt(ni+1);
		}
		else 
		{
			set_inter_cnt(ni+1);
			if(ni == maxni-1)
			{
				#ifdef COOK_LIGHT
				cook_light();
				#else
				cook();
				#endif // COOK_LIGHT
				isppnow=false;
			}
		}
		cout<<"I:"<<get_inter_cnt()<<", R:"<<get_rotate()<<endl;
		// set_rotate(true);
		// isppnow=false;
		ppstoped=false;
	}

	void Checker::cook_light()
	{
		for(auto dish:uc_knowledge){
			cooked_knowledge.resize(1);			
			for(auto uc: dish[0])
			{
				if(backward_first)
					Onp[0].push_back(uc);
				else
					OI[0].push_back(uc);
			}
		}
	}

	void Checker::cook(){
		for(auto dish:uc_knowledge){
			if(cooked_knowledge.size() < dish.size())
				cooked_knowledge.resize(dish.size());
			for(int i = 0; i<dish.size();++i)
			{
				for(auto uc: dish[i])
				{
					int cook_thresh = 2;
					if(uc.size()<cook_thresh)
						// no need for implication analysis, will be changed later.
						cooked_knowledge[i].push_back(uc);
				}
			}
		}
	}

	bool Checker::bi_try_satisfy(Usequence &U, Osequence *O, bool forward, bool&safe_reported)
	{
		// NOTE: can eliminate initialization.
		safe_reported = false;
		Frame Otmp;
		if(!isppnow && O->size()< cooked_knowledge.size())
		{
			Otmp = cooked_knowledge[O->size()];
		}
		MainSolver *main_solver = bi_main_solver;
		// FIXME: only reset when meeting ~p.
		// move into pick_state();
		bi_start_solver->reset();
		int state_level = -1;
		/**
		 * this procedure is like the old car procedure, but the Osequence is not bound to be OI or Onegp.
		 * @param missionary the state to be checked 
		 */

	#ifndef FRESH_U
		Usequence Uold = U;
	#else
		Usequence &Uold= U;
	#endif

	#ifdef RANDOM_PICK
		while(State* missionary = pick_state_random(Uold,state_level))
	#elif defined(PICK_DESC_INC)
		while(State* missionary = pick_state_descendant_inc(Uold,state_level))
	#elif defined(PICK_DESC_DEC)
		while(State* missionary = pick_state_descendant_dec(Uold,state_level))
	#elif defined(PICK_CHILD_INC)
		while(State* missionary = pick_state_children_inc(Uold,state_level))
	#elif defined(PICK_CHILD_DEC)
		while(State* missionary = pick_state_children_dec(Uold,state_level))
	#elif defined(PICK_DESC_INC_ANC)
		while(State* missionary = pick_state_descendant_inc_anc(Uold,state_level))
	#elif defined(PICK_DESC_DEC_ANC)
		while(State* missionary = pick_state_descendant_dec_anc(Uold,state_level))
	#elif defined(PICK_CHILD_INC_ANC)
		while(State* missionary = pick_state_children_inc_anc(Uold,state_level))
	#elif defined(PICK_CHILD_DEC_ANC)
		while(State* missionary = pick_state_children_dec_anc(Uold,state_level))
	#elif defined(PICK_DESC_INC_HEAVY)
		while(State* missionary = pick_state_descendant_inc_heavy(Uold,state_level))
	#elif defined(PICK_DESC_DEC_HEAVY)
		while(State* missionary = pick_state_descendant_dec_heavy(Uold,state_level))
	#elif defined(PICK_RESTART)
		while(State* missionary = pick_state_restart(Uold,state_level))
	#elif defined(PICK_PARTITION_INC)
		while(State* missionary = pick_state_partition_inc(Uold,state_level))
	#elif defined(PICK_PARTITION_DEC)
		while(State* missionary = pick_state_partition_dec(Uold,state_level))
	#elif defined(PICK_ONESHOT)
		while(State* missionary = pick_state_oneshot(Uold,state_level))
	#else
		while(State* missionary = pick_state(Uold,state_level))
	#endif
		{
			LOG("Pick "<<missionary->id);
			/**
			 * build a stack <state, from_level, target_level>
			 */
			CONTAINER stk;
			stk.push(item(missionary,state_level, O->size()-1));
			while (!stk.empty())
			{
				State *s; int dst, src;
				std::tie(s, src, dst) = stk.top();
				LOG("Try " << s->id << " " << dst);
				// bi_main_solver->print_last_3_clauses();
				if (blocked_in(s, dst + 1, O, Otmp))
				{
					stk.pop();
					LOG("Tried before");
					direct_blocked_counter++;
					blocked_ids.insert(s->id);
					
					int new_level = min_not_blocked(s, dst + 2, O->size()-1, O, Otmp);
					if (new_level <= O->size())
					{
						stk.push(item(s, src, new_level-1));
						LOG("Again " << s->id << " " << dst << " " << new_level-1);
						DIVE_DRAW(s,dst+1, s, new_level,1,0);
					}
					else DIVE_DRAW(s,dst+1,nullptr,0,1,0);
					continue;
				}
				
				if(isppnow)
				{
					static int sat_counter = 0;
					static clock_high sat_timer = high_resolution_clock::now();
					sat_counter++;

					#ifdef PP_LIMIT_CNT
					if(sat_counter > PP_LIMIT_CNT)
					{
						ppstoped=true;
					}
					#endif // SAT_LIMIT_CNT

					#ifdef PP_LIMIT_TIME
					if(duration_cast<milliseconds>(high_resolution_clock::now() - sat_timer).count() > PP_LIMIT_TIME * 1000)
					{
						ppstoped=true;
					}
					#endif
					
					if(ppstoped)
					{	
						sat_counter = 0;
						sat_timer=high_resolution_clock::now();
						return true;
					}
				}

				if (sat_assume(bi_main_solver, O, s, dst, Otmp))
				{
					LOG("Succeed");
					if (dst == -1)
						return true;

					State *tprime = getModel(bi_main_solver);
					LOG("Get " << tprime->id << " " << dst);

					#ifdef RECENT
					auto & rct = recent[dst];
					rct = intersect(rct,tprime->s());
					if(rct.empty())
					{
						rct = tprime->s();
					}
					#endif

				#ifdef ORDERED_U
					update_U(U, tprime, dst, s);
				#else
					update_U(U, tprime, src + 1, s);
				#endif

					int new_level = min_not_blocked(tprime, 0, dst-1, O, Otmp);
					if (new_level <= dst) // if even not one step further, should not try it
					{	
						stk.push(item(tprime, src + 1, new_level - 1));
						LOG("Jump " << dst << " " << new_level-1);
						// if(dst != new_level)
						// 	cerr<<"dst: "<<dst<<", new_level : "<<new_level<<endl;
						DIVE_DRAW(s,dst+1, tprime, new_level,(O_level_repeat[dst] != s->id ? 0 : 3),O_level_repeat[dst] == s->id ? O_level_repeat_counter[dst]+1 : 0);
					}
					if(dst >=0)
					{
						if(O_level_repeat[dst] == s->id)
							O_level_repeat_counter[dst]++;
						else
						{
							O_level_repeat[dst] = s->id;
							O_level_repeat_counter[dst] = 1;
						}
					}
				}
				else
				{
					LOG("Fail ");
					stk.pop();

					update_O(O, dst, Otmp, safe_reported);
					if (safe_reported)
						return true;

					int new_level = min_not_blocked(s, dst + 2, O->size()-1, O, Otmp);
					if (new_level <= O->size())
					{
						stk.push(item(s, src, new_level-1));
						LOG("Again " << s->id << " " << dst << " " << new_level-1);
						DIVE_DRAW(s,dst+1, s, new_level,((dst==-1 || O_level_fresh[dst]) ? 0 : 2),(dst==-1 || O_level_fresh[dst]) ? 0 : (O_level_fresh_counter[dst]));
					}
					else {
						DIVE_DRAW(s,dst+1,nullptr,0,((dst==-1 || O_level_fresh[dst]) ? 0 : 2),(dst==-1 || O_level_fresh[dst]) ? 0 : (O_level_fresh_counter[dst]));
					}
					// this is now tried~
					if(dst >= 0)
					{
						if(O_level_fresh[dst])
						{
							O_level_fresh[dst] =false;
							O_level_fresh_counter[dst] = 1;	
						}
						else{
							O_level_fresh_counter[dst]++;	
						}
					}
				}

			}
		}

		// this is same with extend_F_sequence in old CAR.
		O->push_back(Otmp);
		O_level_fresh.push_back(true);
		O_level_fresh_counter.push_back(0);
		O_level_repeat.push_back(-2);
		O_level_repeat_counter.push_back(0);

		spliter.push_back(State::next_id_);
		blocked_counter_array.push_back(direct_blocked_counter);
		direct_blocked_counter = 0;
		#ifdef ROTATE
		if(rotate_enabled)
			rotates.push_back(rotate);
		#endif //ROTATE
		#ifdef MORE_INTER
		asmsubs.push_back(asmsub);
		asmsub.clear();
		#endif //MORE_INTER
		bi_main_solver->bi_add_new_frame(Otmp,O->size()-1,O,forward);
		#ifndef FRESH_UC
		for(auto &frame : *O)
		{
			sort(frame.begin(),frame.end(),[](const Cube&a, const Cube&b){return a.size() < b.size();});
		}
		#endif
		PRINTIF_PROOF();
		return false;
	}

	bool Checker::inv_try_satisfy(State* missionary, Usequence &U, Osequence *O, int level, bool&safe_reported, InvSolver * inv_solver)
	{
		safe_reported = false;
		bool forward = !backward_first;
		// just an empty Otmp is enough.
		Frame Otmp;
		MainSolver *main_solver = bi_main_solver;
		int state_level = -1;

		CONTAINER stk;
		stk.push(item(missionary,state_level, level));
		bool res = false;
		while (!stk.empty())
		{
			State *s; int dst, src;
			std::tie(s, src, dst) = stk.top();
			if (blocked_in(s, dst + 1, O, Otmp))
			{
				stk.pop();
				
				// should not try levels lagrger than original
				int new_level = min_not_blocked(s, dst + 2, level, O, Otmp);
				if (new_level <= level)
				{
					stk.push(item(s, src, new_level-1));
				}
				continue;
			}

			if (sat_assume(bi_main_solver, O, s, dst, Otmp))
			{
				if (dst == -1)
				{
					res = true;
					break;
				}

				State *tprime = getModel(bi_main_solver);
				inv_solver->inv_update_U(U, tprime, src + 1, s);

				int new_level = min_not_blocked(tprime, 0, dst-1, O, Otmp);
				if (new_level <= dst) // if even not one step further, should not try it
				{	
					stk.push(item(tprime, src + 1, new_level - 1));
					LOG("Jump " << dst << " " << new_level-1);
				}
			}
			else
			{
				stk.pop();

				update_O(O, dst, Otmp, safe_reported);
				// inv solver needs to be updated
				bool cons=true;
				// TODO: if cons not true, add to solver
				Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
				std::sort(uc.begin(),uc.end(),car::comp);
				inv_solver->add_uc(uc, dst+1,level);

				if (safe_reported)
					return true;

				int new_level = min_not_blocked(s, dst + 2, level, O, Otmp);
				if (new_level <= level)
				{
					stk.push(item(s, src, new_level-1));
				}
			}
		}

		// We build a tree of states here. But when adding to the opposite U sequence, not all the states are needed. If one state has multiple successors, then there will be some states that have multiple predecessors. It does not help to guide.

		if(res && enable_bi_check)
		{
			std::set<State*> used;
			State* c = which_counter_start();
			int other_level = 0; // start from 0
			State* pr = nullptr;
			while(c)
			{
				update_U(other_U(),c,other_level,pr);
				++other_level;
				pr = c;
				c = inv_solver->prior[c];
				used.insert(c);
			}
			inv_solver->prior.clear();
		}
		return res;
	}

	bool Checker::common_try_satisfy(Usequence &U, Osequence *O, bool forward, bool&safe_reported)
	{
		// NOTE: can eliminate initialization.
		safe_reported = false;
		Frame Otmp;
		MainSolver *main_solver = bi_main_solver;
		bi_start_solver->reset();
		int state_level = -1;
		/**
		 * this procedure is like the old car procedure, but the Osequence is not bound to be OI or Onegp.
		 * @param missionary the state to be checked 
		 */

	#ifndef FRESH_U
		Usequence Uold = U;
	#else
		Usequence &Uold= U;
	#endif

		while(State* missionary = pick_state(Uold,state_level))
		{
			LOG("Pick "<<missionary->id);
			/**
			 * build a stack <state, from_level, target_level>
			 */
			CONTAINER stk;
			stk.push(item(missionary,state_level, O->size()-1));
			std::unordered_map<int, vector<State*>> state_of_level;
			state_of_level[O->size()-1].push_back({missionary});
			while (!stk.empty())
			{
				State *s; int dst, src;
				std::tie(s, src, dst) = stk.top();
				LOG("Try " << s->id << " " << dst);
				if (blocked_in(s, dst + 1, O, Otmp))
				{
					// is it bound to be back?
					assert(state_of_level[dst].back()==s);
					stk.pop(); state_of_level[dst].pop_back();

					LOG("Tried before");
					direct_blocked_counter++;
					blocked_ids.insert(s->id);
					
					int new_level = min_not_blocked(s, dst + 2, O->size()-1, O, Otmp);
					if (new_level <= O->size())
					{
						stk.push(item(s, src, new_level-1)); state_of_level[new_level-1].push_back(s);
						LOG("Again " << s->id << " " << dst << " " << new_level-1);
						DIVE_DRAW(s,dst+1, s, new_level,1,0);
					}
					else DIVE_DRAW(s,dst+1,nullptr,0,1,0);
					continue;
				}

				// TODO: how to make use of how many when succeed?
				// TODO: do some statistical analysis about how many 
				int how_many_comrade = 0;
				if (sat_assume_common(bi_main_solver, O, s, dst, Otmp,state_of_level[dst],how_many_comrade))
				{
					LOG("Succeed");
					if (dst == -1)
						return true;

					State *tprime = getModel(bi_main_solver);
					LOG("Get " << tprime->id << " " << dst);

				#ifdef ORDERED_U
					update_U(U, tprime, dst, s);
				#else
					update_U(U, tprime, src + 1, s);
				#endif

					int new_level = min_not_blocked(tprime, 0, dst-1, O, Otmp);
					if (new_level <= dst) // if even not one step further, should not try it
					{	
						stk.push(item(tprime, src + 1, new_level - 1));state_of_level[new_level-1].push_back(tprime);
						LOG("Jump " << dst << " " << new_level-1);
						DIVE_DRAW(s,dst+1, tprime, new_level,0,0);
					}
				}
				else
				{
					LOG("Fail ");
					assert(state_of_level[dst].back()==s);
					stk.pop();state_of_level[dst].pop_back();
					if(how_many_comrade > 0)
					{
						std::unordered_set<int> comrades;
						for(int i = 0; i< how_many_comrade; ++i)
						{
							comrades.insert(state_of_level[dst][state_of_level[dst].size() - 1 - i]->id);
						}
						stack<item> tmp_stk;
						int seen_counter = 0;
						while(seen_counter < how_many_comrade)
						{
							State *this_s; int this_dst, this_src;
							std::tie(this_s, this_src, this_dst) = stk.top();
							if(this_dst != dst || comrades.count(this_s->id) == 0)
							{
								tmp_stk.push(stk.top());
								stk.pop();
							}
							else
							{
								++seen_counter;
								assert(state_of_level[dst].back()==this_s);
								stk.pop();state_of_level[dst].pop_back();
							}
						}
						while(!tmp_stk.empty())
						{
							stk.push(tmp_stk.top());
							tmp_stk.pop();
						}	
					}

					update_O(O, dst, Otmp, safe_reported);
					if (safe_reported)
						return true;

					int new_level = min_not_blocked(s, dst + 2, O->size()-1, O, Otmp);
					if (new_level <= O->size())
					{
						stk.push(item(s, src, new_level-1));state_of_level[new_level-1].push_back(s);
						LOG("Again " << s->id << " " << dst << " " << new_level-1);
						DIVE_DRAW(s,dst+1, s, new_level,0,0);
					}
					else {
						DIVE_DRAW(s,dst+1,nullptr,0,0,0);
					}
				}
			}
		}

		// this is same with extend_F_sequence in old CAR.
		O->push_back(Otmp);
		spliter.push_back(State::next_id_);
		blocked_counter_array.push_back(direct_blocked_counter);
		direct_blocked_counter = 0;
		#ifdef ROTATE
		if(rotate_enabled)
			rotates.push_back(rotate);
		#endif //ROTATE
		#ifdef MORE_INTER
		asmsubs.push_back(asmsub);
		asmsub.clear();
		#endif //MORE_INTER
		bi_main_solver->bi_add_new_frame(Otmp,O->size()-1,O,forward);
		#ifndef FRESH_UC
		for(auto &frame : *O)
		{
			sort(frame.begin(),frame.end(),[](const Cube&a, const Cube&b){return a.size() < b.size();});
		}
		#endif
		PRINTIF_PROOF();
		return false;
	}

	/**
	 * @brief 
	 * 
	 * @pre level less than check_level has been checked before
	 * @param O 
	 * @param check_level 
	 * @return true : invariant found at target level
	 * @return false : invariant does not exists at target level
	 */
	bool Checker::bi_invariant_found_at(Osequence& O, int check_level,int minimal_update_level, InvSolver *inv_solver)
	{
		// a portion of `invariant_found()`
		if(check_level < minimal_update_level)
		{
			inv::inv_solver_add_constraint_or(O[check_level],check_level, inv_solver);
			return false;
		}
		inv::inv_solver_add_constraint_and(O[check_level], check_level, inv_solver);
		
		bool res = !inv_solver->solve_with_assumption();

		#ifdef INV_HEAVY
		while(!res)
		#elif defined(DINV_MEDIUM)
		int max_try = check_level;
		if(!res && --max_try )
		#elif defined(INV_LIGHT)
		int max_try = 1;
		if(!res && max_try--)
		#else
		while(false)
		#endif
		{
			// This solution is a state, that is in O[check_level], but not in any prior level.
			// We should test whether it can really reach the target, namely ~p for backward-CAR.
			// If so, update them into U sequence of the opposite direction
			// If not, refine the O sequence. 
			// it's like try_satisfy, but it differs in that it should keep a new 'U'.
			State* s = inv_solver->get_state(!backward_first); // get the state, which is in Oj but not in O0....Oj-1.
			clear_defer(s);

			Usequence Utmp = {{s}};
			bool safe_reported;
			
			bool s_reachable = inv_try_satisfy(s, Utmp,&O,check_level-1,safe_reported, inv_solver);
			if(s_reachable)
			{
				// opposite U is extended.
				res = false;
				// break;
			}
			else{
				// TODO: add uc implication analysis in inv_solver.
				// refinement of O sequence is done in inv_try_satisfy
				if(safe_reported)
				{
					res = false;
					// break;
				}
				res = !inv_solver->solve_with_assumption();
				#ifdef PRINT_INV
				cout<<"level = "<< check_level <<", inv found ?= "<<res<<", frame size = "<<O[check_level].size()<<endl;
				#endif
			}
		}
		inv::inv_solver_release_constraint_and(inv_solver,check_level);
		inv::inv_solver_add_constraint_or(O[check_level], check_level, inv_solver);		
		return res;
	}

	// TODO: how to reduce the cost?
    void Checker::simplify_U_children()
    {
		Usequence &U = which_U();
		// how to determine a good thresh?
		#ifdef SIMPU_THRESH
		static int thresh = SIMPU_THRESH;
		#else
		static int thresh = 3;
		#endif // SIMPU_THRESH
		#ifdef SIMPU_INC
		++thresh;
		#endif // SIMPU_INC
		std::map<State*, int> next_count;
		std::unordered_set<State*> to_be_removed;
		for(auto &pair: which_prior())
		{
			next_count[pair.second]++;
		}
		
		for(auto &pair:next_count)
		{
			if(pair.second > thresh)
			{
				to_be_removed.insert(pair.first);
			}
		} 
		cout<<"to be removed size: "<<to_be_removed.size()<<endl;
		// leave U[0] alone.
		for(int i = 1 ; i< U.size(); ++i)
		{
			auto &states = U[i];
			states.erase(std::remove_if(states.begin(), states.end(),
										[&](State *state)
										{
											return to_be_removed.count(state) > 0;
										}),
						 states.end());
		}
	}

	void dfs_simplify(State* state, std::map<State*, std::vector<State*>>& children, std::map<State*, int>& descendant_count) {
		for (State* child : children[state]) {
			dfs_simplify(child,children, descendant_count);
			descendant_count[state] += descendant_count[child] + 1;
		}
	}

    void Checker::simplify_U_descendant()
    {
		Usequence &U = which_U();
		// how to determine a good thresh?
		#ifdef SIMPU_THRESH
		static int thresh = SIMPU_THRESH;
		#else
		static int thresh = 16;
		#endif // SIMPU_THRESH

		#ifdef SIMPU_INC
		thresh <<= 1;
		#endif // SIMPU_INC
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		for(auto &pair: which_prior())
		{
			children[pair.second].push_back(pair.first);
		}

		for (auto& pair : children) {
			dfs_simplify(pair.first,children, descendant_count);
		}

		std::unordered_set<State*> to_be_removed;


		for (auto& pair : descendant_count) {
			if (pair.second > thresh) {
				to_be_removed.insert(pair.first);
			}
		}
		cout<<"to be removed size: "<<to_be_removed.size()<<endl;
		// leave U[0] alone.
		for(int i = 1 ; i< U.size(); ++i)
		{
			auto &states = U[i];
			states.erase(std::remove_if(states.begin(), states.end(),
										[&](State *state)
										{
											return to_be_removed.count(state) > 0;
										}),
						 states.end());
		}
    }

    bool Checker::bi_invariant_found(Osequence* o)
	{
		Osequence &O = *o;
		bool res = false;
		// FIXME: Should we reuse inv_solver instead of recreating?
		InvSolver *inv_solver = new InvSolver(model_);
		// FIXED: shall we start from 0 or 1? 
		// 0, to add O[0] into solver.

		/**
		 * @brief About minimal update level: 
		 * 		It is related to specific O sequence. 
		 * 		Each time invariant check is done, minimal update level is reset to the highest level. 
		 * 		Each time a modification of low-level frame will possibly modify this minimal level.
		 * 
		 */
		for(int i = 0 ; i < O.size(); ++i)
		{
			if(bi_invariant_found_at(O,i,fresh_levels[o],inv_solver))
			{
				#ifdef INV_PRINT
				cout<<"invariant found at "<<i<<endl;
				for(int j = 0; j<=i ;++j)
				{
					cout<<"level = "<<j<<endl;
					cout<<"uc := "<<endl;
					for(auto uc:O[j])
					{
						cout<<"(";
						for(int k:uc)
							cout<<k<<", ";
						cout<<")"<<endl;
					}
				}
				#endif
				while(O.size() > i)
					O.pop_back();
				res = true;
				// already found invariant.
				fresh_levels[o] = -1;
				break;
			}
		}
		//NOTE: not O.size()-1. because that level is also checked.
		fresh_levels[o] = o->size();
		delete(inv_solver);
		#ifdef PRINT_INV
		cout<<"END OF ONE ROUND"<<endl<<endl;
		#endif
		return res;
	}


	Osequence* Checker::create_O_with_state(State* s)
	{
		Osequence *o;
		if (SO_map.find(s) != SO_map.end())
		{
			o = SO_map[s];
		}
		else
		{
			// Cube assigns = s->s();
			// Cube neg_ass = negate(assigns);
			Frame f;
			for(auto i: s->s())
				f.push_back({-i});
			o = new Osequence({f});
			SO_map[s] = o;
			fresh_levels[o] = 0;
			// FIXME: When is the right time to add to solver? And when to set the flag?
			bi_main_solver->bi_add_new_frame(f,o->size()-1, o,!backward_first);
		}
		assert(o);
		return o;
	}

	// NOTE: when we try to pick a guide state, it is not used as part of the assumption but the target. Therefore, uc is not generated according to it, and therefore will not be updated.
	// Luckily, we do not use Onp or OI to guide. Therefore, here we have nothing to do with start_solver.
    State *Checker::pick_state(Usequence &U) 
    {
		static hash_set<unsigned> _seen;
		// NOTE: here, do not directly use Onp or OI.
		for(int i = U.size()-1; i > 0; --i)
		{
			auto frame = U[i];
			for(auto state_ptr:frame)
			{
				if(state_ptr->is_negp)
				{
					continue;
				}
				else if(_seen.count(state_ptr->id) == 0)
				{
					_seen.insert(state_ptr->id);
					return state_ptr;					
				}
			}
		}
		_seen.clear();
        return nullptr;
    }

	State *Checker::pick_state_random(Usequence &U)
    {
		static int index = 0;
		static std::vector<State *> Uset;
		static int maxindex = 0;
		if(index ==0)
		{
			// static std::random_device rd;
			// std::mt19937 g(rd());
			#ifdef RANDSEED
			int seed = RANDSEED;
			#else
			int seed = 6;
			#endif
			std::mt19937 g(seed);
			LOG("[PickGuide] Seed = "<<seed);
			for(int i = 1; i< U.size(); ++i)
			{
				for(auto st:U[i])
				{
					Uset.push_back(st);
				}
			}
			maxindex = Uset.size()-1;
			std::shuffle(Uset.begin(), Uset.end(), g);
		}
		if(index >= maxindex)
		{
			LOG("[PickGuide] End of pick");
			index = 0;
			maxindex = Uset.size();
			Uset.clear();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickGuide] Pick State "<<index<<"/"<<maxindex-1<<", state = "<<p->latches());
		// cerr<<"[PickGuide] Pick State "<<index<<"/"<<maxindex-1<<", state = "<<p->latches()<<" id = "<<int(p->id)<<endl;
		index++;
		return p;
    }

    State *Checker::pick_state(Usequence &U, int& level)
    {
		static hash_set<unsigned> _visited;
		static int record_U=U.size(); // to see if it has changed
		if(record_U>U.size())
		{
			// if it's restarted.
			_visited.clear();
		}
		record_U = U.size();
		for(int i = U.size()-1; i >=0; --i)
		{
			auto frame = U[i];
			for(int j = frame.size() -1; j >=0; --j)
			{ 
				auto state_ptr = frame[j];
				if(state_ptr->is_negp)
				{
					// attention, this function relies on update of solver(add new uc) to work.
					// TODO: look ahead once to avoid invariant check. When start solver can still get a new state, there is no need to check invariant.

					// static State* look_ahead = enumerate_start_state(bi_start_solver);
					State* s = enumerate_start_state(bi_start_solver);
					if(s)
					{
						level = 0;
						// FIXME: whether we should record this.
						// maybe here is just for cex structure.
						update_U(U,s,level,nullptr);
						return s;
					}
					else
						// start states are all tested once.
						continue;
				}
				else if(_visited.find(state_ptr->id) == _visited.end())
				{
					_visited.insert(state_ptr->id);
					level = i;
					return state_ptr;					
				}
			}
		}
		level = -1;
		_visited.clear();
        return nullptr;
    }

	State *Checker::pick_state_random(Usequence &U, int& level)
    {
		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			// static std::random_device rd;
			// std::mt19937 g(rd());
			#ifdef RANDSEED
			int seed = RANDSEED;
			#else
			int seed = 6;
			#endif
			std::mt19937 g(seed);
			LOG("[PickState] Seed = "<<seed);
			std::shuffle(Uset.begin(), Uset.end(), g);
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_inc(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				children[pair.second].push_back(pair.first);
			}

			for (auto& pair : children) {
				dfs_simplify(pair.first,children, descendant_count);
			}
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] < descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second > b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_dec(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				children[pair.second].push_back(pair.first);
			}

			for (auto& pair : children) {
				dfs_simplify(pair.first,children, descendant_count);
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] > descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second > b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_children_inc(Usequence &U, int& level)
    {
		std::map<State*, int> next_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index == 0)
		{
			for(auto &pair: which_prior())
			{
				next_count[pair.second]++;
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&next_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return next_count[a.first] < next_count[b.first] || (next_count[a.first] ==  next_count[b.first] && a.second > b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_children_dec(Usequence &U, int& level)
    {
		std::map<State*, int> next_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				next_count[pair.second]++;
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&next_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return next_count[a.first] > next_count[b.first] || (next_count[a.first] ==  next_count[b.first] && a.second > b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_inc_anc(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				children[pair.second].push_back(pair.first);
			}

			for (auto& pair : children) {
				dfs_simplify(pair.first,children, descendant_count);
			}
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] < descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second < b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_dec_anc(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				children[pair.second].push_back(pair.first);
			}

			for (auto& pair : children) {
				dfs_simplify(pair.first,children, descendant_count);
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] > descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second < b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_children_inc_anc(Usequence &U, int& level)
    {
		std::map<State*, int> next_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				next_count[pair.second]++;
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&next_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return next_count[a.first] < next_count[b.first] || (next_count[a.first] ==  next_count[b.first] && a.second < b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_children_dec_anc(Usequence &U, int& level)
    {

		std::map<State*, int> next_count;

		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			for(auto &pair: which_prior())
			{
				next_count[pair.second]++;
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&next_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return next_count[a.first] > next_count[b.first] || (next_count[a.first] ==  next_count[b.first] && a.second < b.second);} );
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_inc_heavy(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		int thresh = 1024;


		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			// simplify heavily.
			{
				for(auto &pair: which_prior())
				{
					children[pair.second].push_back(pair.first);
				}

				for (auto& pair : children) {
					dfs_simplify(pair.first,children, descendant_count);
				}

				std::unordered_set<State*> to_be_removed;

				for (auto& pair : descendant_count) {
					if (pair.second > thresh) {
						std::queue<State*> tmp;
						tmp.push(pair.first);
						while(!tmp.empty())
						{
							to_be_removed.insert(pair.first);
							State *t = tmp.front();
							tmp.pop();
							for(auto& child : children[t])
							{
								tmp.push(child);
							}
						}
					}
				}

				Uset.erase(std::remove_if(Uset.begin(), Uset.end(),
										[&](std::pair<car::State *, int> p)
										{
											return to_be_removed.count(p.first) > 0;
										}),
				Uset.end());
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] < descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second > b.second);} );
			maxindex = Uset.size();
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_descendant_dec_heavy(Usequence &U, int& level)
    {
		std::map<State*, std::vector<State*>> children;
		std::map<State*, int> descendant_count;

		int thresh = 1024;


		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			{
				for(auto &pair: which_prior())
				{
					children[pair.second].push_back(pair.first);
				}

				for (auto& pair : children) {
					dfs_simplify(pair.first,children, descendant_count);
				}

				std::unordered_set<State*> to_be_removed;

				for (auto& pair : descendant_count) {
					if (pair.second > thresh) {
						std::queue<State*> tmp;
						tmp.push(pair.first);
						while(!tmp.empty())
						{
							to_be_removed.insert(pair.first);
							State *t = tmp.front();
							tmp.pop();
							for(auto& child : children[t])
							{
								tmp.push(child);
							}
						}
					}
				}

				Uset.erase(std::remove_if(Uset.begin(), Uset.end(),
										[&](std::pair<car::State *, int> p)
										{
											return to_be_removed.count(p.first) > 0;
										}),
						Uset.end());
			}
			// NOTE:we should think carefully about, whent the count are the same, which we should try in advance.
			sort(Uset.begin(),Uset.end(),[&descendant_count](const std::pair<State *, int> &a, const std::pair<State *, int>&b){return descendant_count[a.first] > descendant_count[b.first] || (descendant_count[a.first] ==  descendant_count[b.first] && a.second > b.second);} );
			maxindex = Uset.size();
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
		
    }

	State *Checker::pick_state_partition_inc(Usequence &U, int& level)
    {
		static int index = 0;
		static int maxindex = Uset.size();
		static std::map<const State*, int> blocked;
		if(index ==0)
		{
			// copy the States with related to the erased ones.
			for(auto iter = uc_reduce.rbegin(); iter!=uc_reduce.rend(); ++iter)
			{
				auto &blocker = blocked_by[iter->second];
				auto &blockee = blocked_by[iter->first];
				blocker.insert(blocker.end(),blockee.begin(),blockee.end());
			}
			
			
			for(auto &p : blocked_by)
			{
				for(auto &s:p.second)
					blocked[s] = p.first;
			}

			// will be used in later check.
			uc_reduce.clear();
			blocked_by.clear();

			sort(Uset.begin(), Uset.end(), [](std::pair<car::State *, int> &a, std::pair<car::State *, int>&b) {
				return blocked[a.first]  < blocked[b.first];
			});
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			blocked.clear();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
    }

	State *Checker::pick_state_partition_dec(Usequence &U, int& level)
    {
		static int index = 0;
		static int maxindex = Uset.size();
		static std::map<const State*, int> blocked;
		if(index ==0)
		{
			// copy the States with related to the erased ones.
			for(auto iter = uc_reduce.rbegin(); iter!=uc_reduce.rend(); ++iter)
			{
				auto &blocker = blocked_by[iter->second];
				auto &blockee = blocked_by[iter->first];
				blocker.insert(blocker.end(),blockee.begin(),blockee.end());
			}
			
			
			for(auto &p : blocked_by)
			{
				for(auto &s:p.second)
					blocked[s] = p.first;
			}

			// will be used in later check.
			uc_reduce.clear();
			blocked_by.clear();

			sort(Uset.begin(), Uset.end(), [](std::pair<car::State *, int> &a, std::pair<car::State *, int>&b) {
				return blocked[a.first]  > blocked[b.first];
			});
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			blocked.clear();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
    }

	State *Checker::pick_state_restart(Usequence &U, int& level)
    {
		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			// restart
			if(Uset.size() > 1024)
			{
				Uset.clear();
				for(auto &s: which_U()[0])
				{
					Uset.push_back({s,0});
				}
			}
			maxindex = Uset.size();
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
    }

	State *Checker::pick_state_oneshot(Usequence &U, int& level)
    {
		static int index = 0;
		static int maxindex = Uset.size();
		if(index ==0)
		{
			Uset.erase(std::remove_if(Uset.begin(), Uset.end(),
										[&](std::pair<car::State *, int> p)
										{
											if(blocked_ids.count(p.first->id) > 0)
											{
												return true;
											}
											return false;
										}),
						Uset.end());
			for(auto &s: which_U()[0])
			{
				Uset.push_back({s,0});
			}
			maxindex = Uset.size();
		}
		if(index == maxindex)
		{
			LOG("[PickState] End of pick");
			level = -1;
			index = 0;
			maxindex = Uset.size();
			return nullptr;
		}
		auto &p = Uset[index];
		LOG("[PickState] Pick State "<<index<<"/"<<maxindex-1<<", U level = "<<p.second <<", state = "<<p.first->latches());
		index++;
		level = p.second;
		return p.first;
    }

	/**
	 * @brief About the principle here, see newpartialsolver.h
	 * 
	 * @param s 
	 * @param prior_state 
	 */
	State* Checker::get_partial_state(Assignment& s, const State * prior_state)
	{		
		Assignment next_assumptions = s;
		Assignment old_inputs(s.begin(),s.begin() + model_->num_inputs());
		
		if(prior_state)
		// it is not start state
		{
			// negate the s' as the cls, put into the solver.
			Assignment cls = negate(prior_state->s());
			bi_partial_solver->new_flag();
			bi_partial_solver->add_clause_with_flag(cls);
			
			// the assumption being t's input and latches. 
			next_assumptions.push_back(bi_partial_solver->get_flag());
			bi_partial_solver->set_assumption(next_assumptions);
			
			// Therefore, it ought to be unsat. If so, we can get the partial state (from the uc)
			bool res = bi_partial_solver->solve_assumption();
			// uc actually stores the partial state.
			s = bi_partial_solver->get_conflict();
			
			if(res || s.empty())
			{
				// all states can reach this state! It means the counter example is found. Sure the initial state can reach this state.
				// set the next state to be the initial state.
				assert(Ub.size() && Ub[0].size());
				State* init = Ub[0][0];
				s = init->s();
			}

			// block this clause.
			int flag = bi_partial_solver->get_flag();
			bi_partial_solver->add_clause(flag);
			
		}
		else
		// for initial states, there is no such "prior state", only "bad"
		{
			next_assumptions.push_back(-bad_);
			bi_partial_solver->set_assumption(next_assumptions);
			bool res = bi_partial_solver->solve_assumption();
			s = bi_partial_solver->get_conflict_no_bad(-bad_);
			// if one-step reachable, should already been found in immediate_satisfiable.
			assert(!s.empty());
		}

		//FIXME: is this `inputs` really useful?
		Assignment inputs, latches;
		for (auto& it:s){
			if (abs(it) <= model_->num_inputs ())
				inputs.push_back (it);
			else
				latches.push_back (it);
		}
		if(inputs.empty()) // use this inputs as the inputs.
		{
			inputs = old_inputs;	
		}
		
		// this state is not a full state.
		State *pstate = new State(inputs,latches);
		clear_defer(pstate);
		return pstate;

	}


    /**
	 * @brief print the evidence. reuse backward_first, which exactly reveals present searching direciton.
	 * @pre counterexmple is met already.
	 * @param out 
	 */
    void Checker::bi_print_evidence() const
    {
		PRINTIF_PRIOR();
		
		cout << "Counter Example is found in "<< (!backward_first ? "forward" : "backward")<< " search"<<endl;

		// Print Backward chain first.
		bool latch_printed = false;
		
		if(counter_start_b)
		// backward_chain is not empty
		{
			State* to_print = counter_start_b;
			std::stack<std::string> helper;
			// if this is just a portion of the chain, there may not be last_inputs.
			// if(!to_print->last_inputs().empty())
			// 	helper.push(to_print->last_inputs());
			while(to_print)
			{
				State* next = prior_in_trail_b.find(to_print) != prior_in_trail_b.end() ? prior_in_trail_b.at(to_print) : nullptr;
				if (next)
					helper.push(to_print->inputs());
				else
					helper.push(to_print->latches());
				to_print = next;
			}
			while(!helper.empty())
			{
				out<<helper.top()<<endl;
				helper.pop();
			}
			latch_printed = true;
		}
		// then print forward chain.
		if(counter_start_f)
		{
			State* to_print = counter_start_f;
			if(!latch_printed)
				out<<to_print->latches()<<endl;
			while(to_print)
			{
				State* next = prior_in_trail_f.find(to_print) != prior_in_trail_f.end() ? prior_in_trail_f.at(to_print) : nullptr;
				// FIXME: Is this right?
				// if(next)
					out<<to_print->inputs()<<endl;
				// else
				// 	out<<to_print->last_inputs()<<endl;
				to_print = next;
			}
		}
	}

	void Checker::bi_draw_graph()
	{
		dot_out << "digraph SearchTree {" << endl;
		// if the counter example is found, draw the end node
		std::set<int> spliters={spliter.begin(),spliter.end()};
		if(which_counter_start())
		{
			dot_out << which_counter_start()->id <<" [style=filled, fillcolor=red];"<<endl;
			if(backward_first)
				dot_out<<which_counter_start()->id <<" -> NOTP [style=dashed] ;"<<endl;
			else
				dot_out<<"I ->" << which_counter_start()->id << "[style=dashed] ;"<<endl;
		}
		for (auto iter:prior_in_trail_b) {
			if(blocked_ids.count(iter.first->id))
				dot_out<< int(iter.first->id) << " [style = dashed];"<<endl;
			if(spliters.count(iter.first->id))
				dot_out<< int(iter.first->id) << " [color=green];"<<endl;
			else
				dot_out<< int(iter.first->id) << " [color=blue];"<<endl;
			if(!iter.second)
				dot_out<<"I ->" << int(iter.first->id) << ";" << endl;
			else
				dot_out << int(iter.second->id ) << " -> " << int(iter.first->id) << ";" << endl;
		}
		for (auto iter:prior_in_trail_f) {
			if(blocked_ids.count(iter.first->id))
				dot_out<< int(iter.first->id) << " [style = dashed];"<<endl;
			if(spliters.count(iter.first->id))
				dot_out<< int(iter.first->id) << " [color=green];"<<endl;
			else
				dot_out<< int(iter.first->id) << " [color=red];"<<endl;
			if(!iter.second)
				dot_out<<int(iter.first->id) << "-> NOTP " << ";" << endl;
			else
				dot_out <<int(iter.first->id)  << " -> " << int(iter.second->id ) << ";" << endl;
		}
		dot_out << "}" << endl;
	}

	void Checker::print_U_shape()
	{
		int total_uc=0;
		cout<<"shape of O array: [";
		for(int i = 0; i< Onp.size();++i)
		{
			cout<<Onp[i].size()<<", ";
			total_uc += Onp[i].size();
		}
		cout<<"]"<<endl;
		int total_states=0;
		cout<<"shape of U array: [";
		for(int i = 0; i< which_U().size();++i)
		{
			total_states+=which_U()[i].size();
			cout<<which_U()[i].size()<<", ";
		}
		cout<<"]"<<endl;
		cout<<"Total states: "<<total_states<<endl;
		cout<<"Total UC: "<<total_uc<<endl;

		cout<<"spliters id array: [";
		for(int spl:spliter)
		{
			cout<<spl<<", ";
		}
		cout<<"]"<<endl;
		cout<<"tried before count array: [";
		for(int cnt:blocked_counter_array)
		{
			cout<<cnt<<", ";
		}
		cout<<"]"<<endl;
		
	}

    void Checker::print_flags(ostream &out)
    {
		out<<endl;
		out << "------ End Printing Flags ------" << endl;
		for(auto p:SO_map)
		{
			out<<"State is: "<<p.first->id<<endl;
			auto& flags = bi_main_solver->flag_of_O;
			if(flags.find(p.second) != flags.end())
			{
				auto& flag_for_this_O = flags[p.second];
				for(int i = 0; i< flag_for_this_O.size();++i)
					out<<i<<":"<<flag_for_this_O[i]<<", ";
			}
			out<<endl;
		}
		out << "------ End Printing Flags ------" << endl<<endl;
    }

    bool Checker::update_U(Usequence &U, State *s, int level, State * prior_state_in_trail)
    {
		while(U.size() <= level)
			U.push_back({});
		
		#ifdef REORDER_FREQ
		while(Fl.size() <= level)
		{
			int sz = Fl.size();
			Fl.push_back({});
			for(int i = 0; i < model_->num_latches();++i)
			{
				Fl[sz][i+model_->num_inputs()] = 0;
				Fl[sz][-(i+model_->num_inputs())] = 0;
			}
		}
		Assignment assign = s->s();
		for(int i = 0; i< assign.size(); ++i)
		{
			int lit = assign[i];
			Fl[level][lit]++;
		}

		#endif

		// Counter Example Issue. 
		// Every time we insert a new state into U sequence, it should be updated.
		{
			which_prior()[s] = prior_state_in_trail;
		}

		// // do we still need here?
		#ifdef CLOSURE
		for(int index = 0; index <= level; ++index)
			for(auto &state: U[index])
			{
				if(state->id == s->id)
				{
					cout<<"same state is seen again at a "<<(index == level ? "same" : "lower") <<" level"<<endl;
		// 				// no need to update. It is already at a lower or equal level.
		// 				// cout<<"same state is seen again at a "<<(index == level ? "same" : "lower") <<" level"<<endl;
		// 				// FIXME: Refactor using 'closure' of states, instead of present division according to reachable steps.
		// 				#ifdef COUNT_POSSIBLE_REDUNDANT
						static int possible_redundant = 0;
						possible_redundant += level - index;
						cout<<"last occur:"<<index<<", present:"<<level<<endl;
						cout<<"possible redundants :"<< possible_redundant<<endl;
		// 				#endif // COUNT_POSSIBLE_REDUNDANT
		// 				// assert(prior_in_trail[state]->id == prior_state_in_trail->id);
		// 				return false;
				}
			}
		#endif

		U[level].push_back(s);

		// #ifdef RANDOM_PICK 
			Uset.push_back({s,level});
		// #endif // RANDOM_PICK
		
		return true;
    }

	static void shuffle(std::vector<int>& vec)
	{
		#ifdef RANDSEED
			int seed = RANDSEED;
		#else
			int seed = 1;
		#endif
		std::mt19937 g(seed);
		std::shuffle(vec.begin(), vec.end(), g);
	}

	static void shuffle(std::vector<std::vector<int>>& vec)
	{
		#ifdef RANDSEED
			int seed = RANDSEED;
		#else
			int seed = 1;
		#endif
		std::mt19937 g(seed);
		std::shuffle(vec.begin(), vec.end(), g);
	}

	// NOTE: this is not useful, maybe because it is far too heavy?
	void Checker::reorder_according_to_Fl(std::vector<int>& vec, int level)
	{
		if(vec.size()<2)
			return;
		if(Fl.size() <= level)
			return;
		auto & which_Fl = Fl[level];
		sort(vec.begin(),vec.end(),[&which_Fl](int a, int b){return which_Fl[a] > which_Fl[b];});		
	}

	void Checker::reorder_seq(std::vector<int> & vec, int state_level)
	{
		#ifdef PRINT_REORDER
		static int flag = 3;
		if(flag)
		{
			cerr<<"Prior vec: ";
			for(int i:vec)
			cerr<<i<<", ";
			cerr<<endl;
		}
		#endif
		#ifdef REORDER_FREQ
			reorder_according_to_Fl(vec,state_level);
		#elif defined(REORDER_RANDOM)
			shuffle(vec);
		#endif
		#ifdef PRINT_REORDER
		if(flag)
		{
			flag--;
			cerr<<"Post vec: ";
			for(int i:vec)
			cerr<<i<<", ";
			cerr<<endl;
		}
		#endif
	}

	static vector<Cube> reorder_assumption(vector<Cube> inter, const Cube& rres, const Cube& rtmp, const Cube& sml, const Cube& s_next, const Cube& pres_next, const Cube & recent, const Cube & more_inter)
	{
		vector<Cube> pref;
		#if defined(ASS_IRRI)
		pref= inter;
		if(pref.size() == 0)
		{
			pref = {rres,rtmp};
		}
		else
		{
			pref.insert(pref.begin()+1,rres);
			pref.insert(pref.begin()+2,rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"IRRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_IIRR)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,rtmp};
		}
		else if(pref.size() == 1)
		{
			pref.insert(pref.begin()+1,rres);
			pref.insert(pref.begin()+2,rtmp);
		}
		else
		{
			pref.insert(pref.begin()+2,rres);
			pref.insert(pref.begin()+3,rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"IIRR:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_IRIR)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,rtmp};
		}
		else if(pref.size() == 1)
		{
			pref.insert(pref.begin()+1,rres);
			pref.insert(pref.begin()+2,rtmp);
		}
		else
		{
			pref.insert(pref.begin()+1,rres);
			pref.insert(pref.begin()+3,rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"IRIR:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_RIRI)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,rtmp};
		}
		else
		{
			pref.insert(pref.begin()+0,rres);
			pref.insert(pref.begin()+2,rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"RIRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_RRII)
		pref = inter;
		pref.insert(pref.begin()+0,rres);
		pref.insert(pref.begin()+1,rtmp);
		#ifdef PRINT_ASS
		cerr<<"RRII:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_RIIR)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,rtmp};
		}
		else if(pref.size() == 1)
		{
			pref.insert(pref.begin()+0,rres);
			pref.insert(pref.begin()+2,rres);

		}
		else
		{
			pref.insert(pref.begin()+0,rres);
			pref.insert(pref.begin()+3,rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"RIIR:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_IRSRI)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,sml,rtmp};			
		}
		else
		{
			pref.insert(pref.begin() + 1, rres);
			pref.insert(pref.begin() + 2, sml);
			pref.insert(pref.begin() + 3, rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"IRSRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_RISRI)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,sml,rtmp};			
		}
		else
		{
			pref.insert(pref.begin() + 0, rres);
			pref.insert(pref.begin() + 2, sml);
			pref.insert(pref.begin() + 3, rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"RISRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		#elif defined(ASS_RSIRI)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {rres,sml,rtmp};			
		}
		else
		{
			pref.insert(pref.begin() + 0, rres);
			pref.insert(pref.begin() + 1, sml);
			pref.insert(pref.begin() + 3, rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"RSIRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif
		
		#elif defined(ASS_SIRRI)
		pref = inter;
		if(pref.size() == 0)
		{
			pref = {sml,rres,rtmp};			
		}
		else
		{
			pref.insert(pref.begin() + 0, sml);
			pref.insert(pref.begin() + 2, rres);
			pref.insert(pref.begin() + 3, rtmp);
		}
		#ifdef PRINT_ASS
		cerr<<"SIRRI:"<<endl;
		for(auto &cu:pref)
         for(int i:cu)
		cerr<<i<<", ";
		cerr<<endl;
		#endif

		#elif defined(ASS_S_NEXT)
			// cerr<<"s_next : "<<s_next.size()<<endl;
			pref = {s_next};
		#elif defined(ASS_PRES_NEXT)
			// cerr<<"pre s_next"<<pres_next.size()<<endl;
			pref = {pres_next};
			
		#elif defined(ASS_NEXT_INTER_S)
			// cerr<<"pick : next inter s 1"<<endl;
			Cube next_inter = intersect(s_next,pres_next);
			pref = {next_inter, s_next, pres_next};
			assert(next_inter.empty() || imply(s_next, next_inter, true));
		#elif defined(ASS_NEXT_INTER_PRES)
			// cerr<<"pick : next inter s 2"<<endl;
			Cube next_inter = intersect(s_next,pres_next);
			pref = {next_inter, pres_next, s_next};
		#elif defined(ASS_INTER_NEXT)
			// cerr<<"pick : next inter s 2"<<endl;
			
			pref = inter;
			pref.push_back(s_next);
		#elif defined(ASS_NEXT_INTER)
			// cerr<<"pick : next inter s 2"<<endl;
			
			pref = inter;
			pref.insert(pref.begin(),s_next);
		#elif defined(ASS_RECENT)
			pref = {recent};
		#elif defined(ASS_RECENT_INTER)
			if(pref.size() == 0)
			{
				pref = {recent};			
			}
			else
			{
				pref = inter;
				pref.insert(pref.begin()+0,recent);
			}
		#elif defined(ASS_INTER_RECENT)
			if(pref.size() == 0)
			{
				pref = {recent};			
			}
			else
			{
				pref = inter;
				pref.insert(pref.end(),recent);
			}

		#elif defined(ASS_SHUFFLE_INTER)
			shuffle(inter);
			pref = inter;		
			pref.push_back(rres);
			pref.push_back(rtmp);
			pref.push_back(sml);

		#else
		pref = inter;
		pref.push_back(rres);
		pref.push_back(rtmp);
		pref.push_back(sml);
		#endif

		#if defined(ASS_MI_LATER)
			if(more_inter.size() > 0)
			{
				vector<Cube> newpref;
				newpref.resize(2*pref.size());
				for(int i = 0; i< pref.size(); ++i)
				{
					if(pref[i].empty()) continue;
					newpref[i+pref.size()] = pref[i];
					newpref[i] = minus(pref[i],more_inter);
				}
				return newpref;
			}
		#endif // ASS_MI_LATER
		return pref;
	}

	static void reorder_assumption_unittest()
	{
		static bool flag = true;
		if(flag)
		{
			flag = false;
			vector<Cube> inter = { {1,1,1}, {2,2,2},{3,3,3} };
			Cube rres = {4,4,4};
			Cube rtmp = {5,5,5};
			Cube sml = {6,6,6};
			cerr<<"enter reorder"<<endl;
			reorder_assumption(inter,rres,rtmp,sml,{},{},{},{});
			exit(1);
		}
	}

	static Cube minus(const Cube& c1, const Cube& c2)
	{
		std::vector<int> res;
		int i = 0, j = 0;
		while (i < c1.size() && j < c2.size()) {
			if (abs(c1[i]) == abs(c2[j])) {
				if(c1[i] != c2[j])
					res.push_back(c1[i]);
				i++;
				j++;
			} else if (abs(c1[i]) < abs(c2[j])) {
				res.push_back(c1[i]);
				i++;
			} else {
				j++;
			}
		}
		while (i < c1.size())
		{
			res.push_back(c1[i]);
			i++;
		}
		return res;
	}

	static Cube intersect(const Cube& c1, const Cube& c2)
	{
		std::vector<int> res;
		int i = 0, j = 0;
		while (i < c1.size() && j < c2.size()) {
			if (abs(c1[i]) == abs(c2[j])) {
				if(c1[i] == c2[j])
					res.push_back(c1[i]);
				i++;
				j++;
			} else if (abs(c1[i]) < abs(c2[j])) {
				i++;
			} else {
				j++;
			}
		}
		return res;
	}

    bool Checker::sat_assume(MainSolver *solver, Osequence *O, State *s, int level, const Frame& Otmp)
    {
		bool res = false;
		if (level == -1)
		{
			// NOTE: in backward CAR, here needs further check.
			if (last_check(s, O))
				res = true; // unsafe
			else
				res = false;
		}
		else
		{
			bool forward = !backward_first;
			vector<Cube> inter;
			bool inter_invalidate = false;
			// FIXME: fill in here.
			#if defined(INTER_CNT_DYN)
			do
			{
				#ifdef INTER
				// intersection:
				const Frame &frame = level+1 < O->size() ? (*O)[level+1] : Otmp;
				int index = 1;
				// mark all the lits in this state, remove them when used
				// cerr<<endl<<endl<<"new round:"<<endl;
				static unordered_set<int> marked;
				marked.clear();
				for(auto &lit: s->s())
					marked.insert(lit);
				while(frame.size() >= index && marked.size())
				{
					Cube inter_next;
					
					for(int i = 0; i< 1 && frame.size() >=index * 1 + i; ++i)
					{
						const Cube& last_uc = frame[frame.size()-index * 1 - i];
						// inter.insert(inter.end(),inter_next.begin(),inter_next.end());
						if(i == 0)
							inter_next = s->intersect(last_uc);
						else
							inter_next = intersect(inter_next,last_uc);
						if(!inter_next.empty() && inter_next.back() != last_uc.back())
						{
							inter_invalidate = true;
						}
					}
					// otherwise, do not do this!
					#ifdef INTER_INVALIDATE_SIMPLE
					if( inter_invalidate && O_level_fresh[level])
					{
						inter_invalidate = false;
						index++;
						continue;
					}
					#elif defined(INTER_INVALIDATE_HARD)
					if(inter_invalidate)
					{
						inter_invalidate = false;
						index++;
						continue;
					}
					#else
					for(auto &lit: inter_next)
					{
						if(marked.count(lit))
						{
							inter.push_back({lit});
							marked.erase(lit);
						}
					}
					#endif
					// inter.push_back(inter_next);
					index++;
				}
				#endif // INTER
			}
			while(0);
			#else
			do
			{
				#ifdef INTER
				// intersection:
				const Frame &frame = level+1 < O->size() ? (*O)[level+1] : Otmp;
				int index = 1;
				while(index <= get_inter_cnt()  && frame.size() >= index * 1)
				{
					Cube inter_next;	
					// <1 : INTER_DEPTH is removed and set to 1 permanently.
					for(int i = 0; i < 1 && frame.size() >=index * 1 + i; ++i)
					{
						#ifdef INTER_RAND
						// the influence of this is not obvious
						#ifdef RANDSEED
						int seed = RANDSEED;
						#else
						int seed =1;
						#endif
						mt19937 mt_rand(seed);
						const Cube& last_uc = frame[mt_rand()%(frame.size())];
						#elif defined(INTER_LONG) 
						int last_uc_index = frame.size() - index;
						if(longest_uc_index.size() > level+2)
							last_uc_index = longest_uc_index[level+1].second;
						const Cube&last_uc = frame[last_uc_index];

						#elif defined(INTER_SHORT) 
						int last_uc_index = frame.size() - index;
						if(shortest_uc_index.size() > level+2)
							last_uc_index = shortest_uc_index[level+1].second;
						const Cube&last_uc = frame[last_uc_index];
						#else
						const Cube& last_uc = frame[frame.size()-index * 1 - i];
						#endif // INTER_RAND
						if(i == 0)
							// this preserves the last bit
							inter_next = s->intersect(last_uc);
						else
							// this does not
							inter_next = intersect(inter_next,last_uc);
						// FIXME: this is ugly. change it later please.
						if(!inter_next.empty() && i == 0 &&  !s->imply({last_uc.back()}) )
						{
							inter_invalidate = true;
						}
					}
					// otherwise, do not do this!
					#ifdef LAST_FIRST
					if(inter_next.size() >1 && !inter_invalidate)
					{
						// insert the last bit to the front.
						inter_next.insert(inter_next.begin(),inter_next.back());
						inter_next.pop_back();
					}
					#elif defined (INTER_REVERSE)
					if(inter_next.size() >1)
					{
						reverse(inter_next.begin(),inter_next.end());
					}
					#elif defined (INTER_LF_SHUFFLE)
					if(inter_next.size() >1)
					{
						int confl_bit = inter_next.back();
						// insert the last bit to the front.
						if(!inter_invalidate)
							inter_next.pop_back();
						shuffle(inter_next);
						if(!inter_invalidate)
							inter_next.insert(inter_next.begin(),confl_bit);
					}
					#elif defined (INTER_SHUFFLE)
					if(inter_next.size() >1)
					{
						if(!inter_invalidate)
							inter_next.pop_back();
						shuffle(inter_next);
					}
					#else
					if(inter_next.size() >1 && !inter_invalidate)
						inter_next.pop_back();
					#endif
					#ifdef INTER_INVALIDATE_SIMPLE
					if( (!inter_invalidate) || O_level_fresh[level])
					{
						inter.push_back(inter_next);
						inter_invalidate = false;
					}
					#elif defined(INTER_INVALIDATE_HARD)
					if( (!inter_invalidate))
					{
						inter.push_back(inter_next);
						inter_invalidate = false;
					}
					#else
					inter.push_back(inter_next);
					#endif
					index++;
				}
				// complete it
				if(inter.empty()) inter.push_back({}); 
				// cout<<"inter size = "<<inter.size()<<" / "<<INTER_CNT<<endl;
				#ifdef SHUFFLE_INTER
				reorder_seq(inter,level + 1);
				#endif
				#endif // INTER
			}
			while(0);

			#endif

				// NOTE: VSIDS is not good.
				// sort(inter.begin(),inter.end(),[&](int a, int b){return freq[a] > freq[b];});

			Cube s_next;
			#ifdef FORESEE
			do
			{
				Cube nexts = s->next_latches();
				s_next = s->intersect(nexts);
			}
			while(0);
			#endif // FORESEE

			Cube pres_next;
			#ifdef FORESEE_PRIME
			do
			{
				if(which_prior().find(s) == which_prior().end())
					break;
				auto pres = which_prior()[s];
				if(pres != nullptr)
				{
					Cube nexts = pres->next_latches();
					pres_next = s->intersect(nexts);
				}
			}
			while(0);
			#endif // FORESEE_PRIME


			
			// cerr<<"common size : ";
			// cerr<<s_next.size()<<endl;

			// cerr<<"inter size : "<<intersect()
			

			Cube rres, rtmp;
			do
			{	
				#ifdef ROTATE
				// rotates[i]
					if(!rotate_enabled)
						break;
					Cube &rcu = level +1 < rotates.size() ? rotates[level+1] : rotate;
					if(rcu.empty())
					{
						rcu = s->s();
						// TODO: try this to be inter?
						break;
					}

					// calculate intersection and put the others behind.
					for(int i = 0; i < rcu.size(); ++i)
					{
						// full state
						if(s->size() == model_->num_latches())
						{
							if(s->element(abs(rcu[i])-model_->num_inputs ()-1) == rcu[i])
								rres.push_back(rcu[i]);
							else
								rtmp.push_back(-rcu[i]);
						}
						else
						// TODO: merge with "intersection"
						// a partial state
						{
							int i = 0, j = 0;
							while (i < s->size() && j < rcu.size())
							{
								if (rcu[j] == s->element(i))
								{
									rres.push_back(rcu[j]);
									i++;
									j++;
								}
								else
								{
									rtmp.push_back(s->element(i));
									if (rcu[j] < s->element(i))
										j++;
									else
										i++;
								}
							}
						}

					}
						// inter ++ (s ∩ rcu) ++ (s - rcu) ++ s

					#ifdef SHUFFLE_ROTATE
						reorder_seq(rres,level + 1);
					#endif
				#endif
			} while (0);
				
			Cube sml;
			do
			{
				#ifdef SML
				if(which_U().size() <= level+1)
					break;
				auto &Uframe =  which_U()[level+1];
				int index = 1;
				while(index <= SML_CNT  && Uframe.size() >= index)
				{	
					State* last_state = Uframe[Uframe.size()-index];
					if(index == 1)
					{
						sml = s->intersect(last_state);
						
					}
					else
					{
						sml = last_state->intersect(sml);
					}
					index++;
				}
				#endif
			} while (0);

			Cube more_inter;
			do
			{
				#ifdef MORE_INTER

				// FIXED: use original inter
				// /**
				//  * @brief Like traditional intersection, put the intersection with last uc in the front.
				//  * 
				//  */
				// const Frame &frame = level+1 < O->size() ? (*O)[level+1] : Otmp;
				// if(frame.empty())
				// 	break;
				
				// const Cube& last_uc = frame.back();
				// more_inter = s->intersect(last_uc);


				const Cube& ref = level +1 < asmsubs.size() ? asmsubs[level+1] : asmsub;
				if(ref.empty())
					break;
				
				// they cannot be uc if Ol is not updated.
				more_inter = s->intersect_no_dupl(ref);
				
				// cout<<"more size : "<<more.size()<<endl;

				// cout<<"---------"<<endl;
				// cout<<"state";
				// for(int i:s->s())
				// cout<<i<<", ";
				// cout<<endl;		

				// cout<<"inter with last_uc";
				// for(int i:s->intersect(last_uc))
				// cout<<i<<", ";
				// cout<<endl;		
				
				// cout<<"inter with last record";
				// for(int i:more)
				// cout<<i<<", ";
				// cout<<endl;		

				// cout<<"---------"<<endl;

				#endif // MORE_INTER
			} while (0);

			Cube rct;
			#ifdef RECENT
			if(recent.size() < level + 1)
				recent.resize(level+1);
			rct = intersect(s->s(),recent[level]);
			#endif // RECENT
			
			vector<Cube> pref = reorder_assumption(inter,rres,rtmp,sml,s_next,pres_next,rct,more_inter);
			solver->bi_set_assumption(O,s,level,forward,pref);
			res = solver->solve_with_assumption();
			#ifdef FALLIN_STATS
			// {
			// 	// pick uc to inter.
			// 	const Frame &frame = level+1 < O->size() ? (*O)[level+1] : Otmp;
			// 	for(auto uc : frame)
			// 	{
			// 		int last_bit = uc.back();
			// 		if(s->intersect({last_bit}).empty())
			// 			cout<<"-";
			// 		else
			// 			cout<<"+";
			// 	}
			// 	cout<<endl;
			// }
			// if(inter_invalidate)
			// 	cout<< s->id <<" to level "<< level <<", invalidate."<<endl;

			if(!res)
			{
				{
				bool cons = true;
				Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
				// Cube prefs;
				// for(auto cu : pref)
					// prefs.insert(prefs.end(),cu.begin(),cu.end());
				
				// cout<<"prefs:";
				// for(int i:prefs)
				// cout<<i<<", ";
				// cout<<endl;
				// cout<<"uc:";
				// for(int i:uc)
				// cout<<i<<", ";
				// cout<<endl;

				stats.nInAss++;				
				if(pref.size() >0 && imply(pref[0],uc, false) && level != 0)
				{
					// if(!O_level_fresh[level] ) 
					{
						// cout<< "last uc:\t";
						// Cube last_uc = ( level+1 < O->size() ? (*O)[level+1] : Otmp).back();
						// for(int i:last_uc)
						// cout<<i<<", ";
						// cout<<endl;

						// cout<< "prefs:\t";
						// for(int i: prefs)
						// cout<<i<<", ";
						// cout<<endl;

						// cout<< "new uc:\t";
						// for(int i:uc)
						// cout<<i<<", ";
						// cout<<endl;
						
						// cout<< s->id <<" to level "<< level <<", fall in pref."<<endl;

						// {
						// 	// try reorder
						// 	uc.insert(uc.begin(),uc.back());
						// 	solver->bi_set_assumption(O,s,level,forward,{uc});
						// 	res = solver->solve_with_assumption();
						// 	Cube newuc = bi_main_solver->get_conflict(!backward_first,cons);	
						// 	cout<<"rev uc : \t";
						// 	for(int i:newuc)
						// 	cout<<i<<", ";
						// 	cout<<endl;
						// }
						stats.nInAss0_succeed++;
					}
				}
				if( pref.size() >1 && imply(pref[1],uc,false) && level != 0){
					stats.nInAss1_succeed++;
					#ifdef ROTATE
					if(rotate_enabled)
					{
						Cube &rcu = level +1 < rotates.size() ? rotates[level+1] : rotate;
						rcu.clear();
					}
					#endif // ROTATE
				}
				// else if( pref.size() >2 && imply(pref[2],uc,false) && level != 0){
				// 	stats.nInAss2_succeed++;
				// }
				}
			}
			#endif

			#ifdef RECENT
			if(!res)
			{
				auto &rct = recent[level];
				bool cons=true;
				Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
				if(imply(rct,uc,true))
				{
					rct.clear();
				}
			}
			#endif // RECENT

			#ifdef ROTATE
			if(!res)
			{
				if(rotate_enabled)
				{
				// Cube st;
				// st.reserve(model->num_latches());
				// // here is not the original inter, but the composite one.
				// Cube original_rotate_vec = inter.size() ? inter[0] : Cube({});
				// original_rotate_vec.insert(original_rotate_vec.end(), rres.begin(),rres.end());
				// original_rotate_vec.insert(original_rotate_vec.end(), rtmp.begin(),rtmp.end());
				// for (int i = original_rotate_vec.size ()-model_->num_latches(); i < original_rotate_vec.size (); ++ i)
		    	// 	st.push_back (original_rotate_vec[i]);
				// rcu = st;
				// assert(rtmp.size() + rres.size() == st.size());

				Cube st1 = rres;
				st1.insert(st1.end(),rtmp.begin(),rtmp.end());
				// for(int i = 0; i < st.size(); ++i)
				// {
				// 	if(st[i] != st1[i])
				// 		assert(false);
				// }
				Cube &rcu = level +1 < rotates.size() ? rotates[level+1] : rotate;
				rcu = st1;
				// rcu = st1.empty() ? s->s() : st1;
				}
			}
			#endif

			// #define MORE_INTER
			#ifdef MORE_INTER 
			if(!res)
			{
				// if not sat, update recording.
				Cube& ref = level +1 < asmsubs.size() ? asmsubs[level+1] : asmsub;
				ref.clear();
				/**
				 * @brief any order of any subsequence of UC cannot be uc, this can be proved according to minisat(glucose)'s assumption logic
				 * 
				 */
				// the first is always the reason for conflict.

				Glucose::Lit l = bi_main_solver->conflict[0];
				int last_ass = - bi_main_solver->lit_id (l);
				
				for(int i = 0 ; i < bi_main_solver->assumption_.size();++i)
				{
					ref.push_back(bi_main_solver->lit_id(bi_main_solver->assumption_[i]));
					if(bi_main_solver->assumption_[i] == l)
						break;
				}				
				
				// test methods
				// cerr<<"id = "<<s->id<<", level = "<<level<<endl;

				// cerr<<"assumptions : ";
				// for (int i = 0; i < bi_main_solver->assumption_.size (); i ++)
	        	// 	cerr << bi_main_solver->lit_id (bi_main_solver->assumption_[i]) << " ";
				// cerr<<endl;
				// Cube uc = bi_main_solver->get_conflict(!backward_first);
				// cerr<<"uc : ";
				// for(int i:uc)
				// cerr<<i<<", ";
				// cerr<<endl;
        		// cerr<<" my guess is "<<- bi_main_solver->lit_id (l)<<endl;


			}
			#endif // MORE_INTER
			PRINTIF_QUERY();
		}		
		PRINTIF_SIMPLE_SAT();

		return res;    
	}

	bool Checker::sat_assume_common(MainSolver *solver, Osequence *O, State *s, int level, const Frame& Otmp, const std::vector<State*>& state_of_level, int& how_many)
    {
		bool res = false;
		if (level == -1)
		{
			// NOTE: in backward CAR, here needs further check.
			if (last_check(s, O))
				res = true; // unsafe
			else
				res = false;
		}
		else
		{
			bool forward = !backward_first;
			vector<Cube> inter;
			do
			{
				#ifdef INTER
				// intersection:
				const Frame &frame = level+1 < O->size() ? (*O)[level+1] : Otmp;
				int index = 1;
				while(index <= get_inter_cnt()  && frame.size() >= index * 1)
				{	
					Cube inter_next;
					
					for(int i = 0; i< 1 && frame.size() >=index * 1 + i; ++i)
					{
						#ifdef INTER_RAND
						// the influence of this is not obvious
						int seed = 1;
						mt19937 mt_rand(seed);
						const Cube& last_uc = frame[mt_rand()%(frame.size())];
						#else
						const Cube& last_uc = frame[frame.size()-index * 1 - i];
						#endif // INTER_RAND
						// inter.insert(inter.end(),inter_next.begin(),inter_next.end());
						if(i == 0)
							inter_next = s->intersect(last_uc);
						else
							inter_next = intersect(inter_next,last_uc);
					}
					inter.push_back(inter_next);
					index++;
				}
				#ifdef SHUFFLE_INTER
				reorder_seq(inter,level + 1);
				#endif
				#endif // INTER
			}
			while(0);
						
						
			// 		}
			// 		else
			// 		{
			// 			sml = last_state->intersect(sml);
			// 		}
			// 		index++;
			// 	}
			// 	#endif
			// } while (0);

			// vector<Cube> pref = reorder_assumption(inter,rres,rtmp,sml);

			// 		}
			// 		else
			// 		{
			// 			sml = last_state->intersect(sml);
			// 		}
			// 		index++;
			// 	}
			// 	#endif
			// } while (0);

			// vector<Cube> pref = reorder_assumption(inter,rres,rtmp,sml);
			Cube common; 
			Cube poss;
			if(state_of_level.size() > 1)
			{
				common = inter.size()? inter[0] : s->s();
				reorder_intersect(common,state_of_level,poss);
			}

			solver->bi_set_assumption(O,s,level,forward,{common});
			
			res = solver->solve_with_assumption();
			if(!res && state_of_level.size() > 1)
			{
				bool cons=true;
				Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
				how_many = reorder_intersect_how_many(common,uc,poss);	
			}
			PRINTIF_QUERY();
		}
		PRINTIF_SIMPLE_SAT();

		return res;    
	}

	State* Checker::getModel(MainSolver * solver)
	{
		bool forward = !backward_first;
		State* s = solver->get_state(forward);
		// NOTE: if it is the last state, it will be set last-inputs later.
		clear_defer(s);
		return s;
	}

	State* Checker::getModel(MainSolver * solver, State* prior)
	{
		bool forward = !backward_first;
		if(!forward)
		{
			State* s = solver->get_state(forward);
			// NOTE: if it is the last state, it will be set last-inputs later.
			clear_defer(s);
			return s;
		}
		else
		{
			#ifdef PARTIAL
				Assignment full = solver->get_state_full_assignment(forward);
				return get_partial_state(full,prior);
			#else
				State* s = solver->get_state(forward);
				clear_defer(s);
				return s;
			#endif
		}
	}

    void Checker::bi_clean()
	{
		for(State* duty: clear_duties)
		{
			if(duty)
			{
				delete duty;
				duty = nullptr;
			}
		}
		for(MainSolver* solver: clear_duties_mainsolver)
		{
			if(solver)
			{
				delete solver;
				solver = nullptr;
			}
		}
		for (auto &os : SO_map)
		{
			if (os.second)
			{
				if(os.second==&Onp || os.second == &OI)
					continue;
				delete os.second;
				os.second = nullptr;
			}
		}
	}

	void Checker::clear_Os(State* s, Osequence *Os)
	{
		assert(s);
		assert(Os);
		Os->clear();
		delete (Os);
		SO_map.erase(s);
	}

	void Checker::block_state_forever(const State* s)
	{
		Cube negstate = negate(s->s());
		bi_main_solver->add_clause(negstate);
		++blocked_count;
		cout<<"blocked_count: "<<blocked_count<<endl;
	}

    void Checker::add_uc_to_solver(Cube& uc, Osequence *O, int dst_level, Frame& Otmp)
    {
		if(dst_level < fresh_levels[O])
			fresh_levels[O] = dst_level;
		Frame &frame = (dst_level < int(O->size())) ? (*O)[dst_level] : Otmp;
		// To add \@ cu to \@ frame, there must be
		// 1. \@ cu does not imply any clause in \@ frame
		// 2. if a clause in \@ frame implies \@ cu, replace it by \@cu

		#ifdef FRESH_UC		
		Frame tmp_frame;
		#ifdef INTER_LONG
		if(longest_uc_index.size()<dst_level+1)
			longest_uc_index.resize(dst_level+1);
			int max_len =0, max_index = -1;
		#endif
		#ifdef INTER_SHORT
		if(shortest_uc_index.size()<dst_level+1)
			shortest_uc_index.resize(dst_level+1);
			int min_len =uc.size(), min_index = -1;
		#endif
		for (int i = 0; i < frame.size(); i++)
		{
			if (!imply(frame[i], uc,true))
			{
				tmp_frame.push_back(frame[i]);
				#ifdef INTER_LONG
				if(max_len <= frame[i].size())
				{
					max_len = frame[i].size();
					max_index = tmp_frame.size()-1;
				}
				#endif // INTER_LONG
				#ifdef INTER_SHORT
				if(min_len >= frame[i].size())
				{
					min_len = frame[i].size();
					min_index = tmp_frame.size()-1;
				}
				#endif // INTER_SHORT
			}
			#ifdef PICK_PARTITION_INC
			else
				uc_reduce[get_uc_id(frame[i])] = get_uc_id(uc);
			#endif
		}
		tmp_frame.push_back(uc);

		#ifdef INTER_LONG
		if(max_len <= uc.size())
		{
			max_len = uc.size();
			max_index = tmp_frame.size()-1;
		}
		longest_uc_index[dst_level] = std::pair<int,int>({max_len,max_index});
		#endif // INTER_LONG
		
				
		#ifdef INTER_SHORT
		if(min_len >= uc.size())
		{
			min_len = uc.size();
			min_index = tmp_frame.size()-1;
		}
		shortest_uc_index[dst_level] = std::pair<int,int>({min_len,min_index});
		#endif // INTER_SHORT
		
		frame = tmp_frame;
		#else
		frame.push_back(uc);
		#endif // FRESH_UC

		if(dst_level < O->size())
			bi_main_solver->bi_add_clause_from_cube(uc,dst_level,O,!backward_first);
		else if(dst_level == O->size())
		{
			if(!backward_first)
			{
				// FIXME: test me later.
				// Not always. Only if the start state is ~p.
				bi_start_solver->add_clause_with_flag(uc);
			}
		}

    }

    void Checker::update_O(Osequence *O, int dst, Frame &Otmp, bool& safe_reported)
    {
		bool cons=true;
		Cube uc = bi_main_solver->get_conflict(!backward_first,cons);
		
		// uc is not just latches.
		if(!cons)
		{
			// cerr<<"This is strange"<<endl;
			// Cube permanentuc = negate(bi_main_solver->get_uc());
			// bi_main_solver->add_clause(permanentuc);
			// bi_start_solver->add_clause(permanentuc);
			cerr<<"This is strange"<<endl;
			cout<<"This is strange"<<endl;
			auto cu = bi_main_solver->get_uc();
			cout<<"uc from solver:";
			for(int i:cu)
			cout<<i<<", ";
			cout<<endl;
			// bi_main_solver->clear_assumption();
			// for(auto p: bi_main_solver->get_uc ())
			// {
			// 	bi_main_solver->assumption_.push (bi_main_solver->SAT_lit (p));
			// }
			// bool res = bi_main_solver->solve_with_assumption();
			// cout<<"res is "<<res<<endl;
			// exit(1);

			// cout<<"This is strange"<<endl;
			// bi_main_solver->clear_assumption();
			// auto cu = bi_main_solver->get_uc();
			// Cube cl;
			// for (int i = 0; i < cu.size (); i ++)
			// {
			// 	if( abs(cu[i]) <= model_->num_inputs() + model_->num_latches()+1 )
			// 		cl.push_back(model_->prime (cu[i]));
			// 		// cl.push_back((cu[i]));
			// 	else
			// 		cl.push_back(cu[i]);
			// }
			// // bi_main_solver->set_assumption (cl);
			// for(auto p: cl)
			// {
			// 	bi_main_solver->assumption_.push (bi_main_solver->SAT_lit (p));
			// }
			// bool res = bi_main_solver->solve_with_assumption();
			// cout<<"res is "<<res<<endl;
			// exit(1);


			//add this uc permanently
			// bi_main_solver->clear_assumption();
			// for(auto p:bi_main_solver->get_uc())
			// {
			// 	bi_main_solver->assumption_.push (bi_main_solver->SAT_lit (p));
			// }
			// bool res = bi_main_solver->solve_with_assumption();
			// cout<<"res is "<<res<<endl;
			// should we add this to inv solver?
			
			// bi_main_solver->add_clause(permanentuc);
		}

		if(uc.empty())
		{
			// this state is not reachable?
			//FIXME: fix here. It is not really blocked forever.
			PRINTIF_UNREACHABLE();
			safe_reported = true;
		}
		
		add_uc_to_solver(uc,O,dst+1,Otmp);
		O_level_fresh[dst+1]=true;
		O_level_fresh_counter[dst+1] = 0;
		
    }

    bool Checker::bi_init_sequences(bool& res)
    {
		State *init = new State(model->init());
		Frame O0; // a frame with only one uc.
		if(immediate_check(init,bad_,res,O0))
			return true;

		// forward inits.
		{
			// Uf[0] = ~p;

			// NOTE: we do not explicitly construct Uf[0] data strcutre. Because there may be too many states.  We actually need concrete states to get its prime. Therefore, we keep an StartSolver here, and provide a method to enumerate states in Uf[0].
			// construct an abstract state
			State* negp = new State(true);
			prior_in_trail_f[negp] = nullptr;
			clear_defer(negp);
			Uf = {{negp}};

			// O_{\neg P} used in backward search

			// clauses will be added by immediate_satisfible.
			// SAT_assume(init, ~p)
			// uc from init
			// ~p in ~uc
			// use uc to initialize O[0] is suitable. 
			Onp = Osequence({O0});
			SO_map[negp] = &Onp;
		}
		// backward inits.
		{
			// Ub[0] = I;
			clear_defer(init);
			// Ub = {{init_}};
			update_U(Ub,init,0,nullptr);

			// O_I
			// NOTE: O stores the UC, while we actually use negations.
			Frame frame;
			for(auto lit: init->s())
			{
				frame.push_back({-lit});
			}
			OI = {frame};
			SO_map[init]=&OI;
		}
		#ifdef ROTATE
		rotates.push_back(init->s());
		#endif

		if(backward_first || enable_bi_check)
			bi_main_solver->bi_add_new_frame(Onp[0],Onp.size()-1, &Onp,false);
		if(!backward_first || enable_bi_check)
			bi_main_solver->bi_add_new_frame(OI[0],OI.size() - 1,&OI,true);
		return false;
	}

	bool Checker::inv_init_sequences(bool& res)
    {
		State *init = new State(model->init());
		State* negp = new State(true);
		clear_defer(init);
		clear_defer(negp);
		Frame O0; // a frame with only one uc.
		if(immediate_check(init,bad_,res,O0))
			return true;

		// forward inits.
		if(!backward_first || enable_bi_check)
		{
			// Uf[0] = ~p;

			// NOTE: we do not explicitly construct Uf[0] data strcutre. Because there may be too many states.  We actually need concrete states to get its prime. Therefore, we keep an StartSolver here, and provide a method to enumerate states in Uf[0].
			// construct an abstract state
			Uf = {{negp}};
			prior_in_trail_f[negp] = nullptr;


			// O_I
			// NOTE: O stores the UC, while we actually use negations.
			Frame frame;
			for(auto lit: init->s())
			{
				frame.push_back({-lit});
			}
			OI = {frame};
			SO_map[init]=&OI;
		}
		// backward inits.
		if(backward_first || enable_bi_check)
		{
			// Ub[0] = I;
			update_U(Ub,init,0,nullptr);

			// Ob[0] = uc0.
			// clauses will be added by immediate_satisfible.
			// SAT_assume(init, ~p)
			// uc from init
			// ~p in ~uc
			// use uc to initialize O[0] is suitable. 
			Onp = Osequence({O0});
			SO_map[negp] = &Onp;

		}
		#ifdef ROTATE
		rotates.push_back(init->s());
		#endif

		if(backward_first|| enable_bi_check)
			bi_main_solver->bi_add_new_frame(Onp[0],Onp.size()-1, &Onp,false);
		if(!backward_first|| enable_bi_check)
			bi_main_solver->bi_add_new_frame(OI[0],OI.size() - 1,&OI,true);

		O_level_fresh={true};
		O_level_fresh_counter={0};
		O_level_repeat={-2};
		O_level_repeat_counter={0};
		return false;
	}

	//////////////helper functions/////////////////////////////////////////////
	Checker::Checker(Model *model,std::ostream &out,std::ofstream & trail_out,std::ofstream & dot_out, std::ofstream &dive_out, bool enable_dive, bool forward, bool propagate, bool evidence, int index_to_check,bool enable_bi):model_(model),out(out),dot_out(dot_out), dive_out(dive_out), enable_dive(enable_dive), evidence_(evidence),enable_bi_check(enable_bi),trail_out(trail_out)
	{
		// TODO: use propagate to accelerate
		backward_first = !forward;
		bi_main_solver = new MainSolver(model);
		#ifdef REDUCE_LEARNT_BY
		int reduceby = REDUCE_LEARNT_BY;
		#else
		int reduceby = 2;
		#endif
		bi_main_solver->setReduceBy(reduceby);
		bi_partial_solver = new PartialSolver(model);
		// TODO: extend to multi-properties.
		bad_ = model->output(index_to_check);
		bi_start_solver = new StartSolver(model,bad_,true);
		if(enable_dive)
		{
			assert(dive_out.is_open());
			dive_draw_head();
		}
		stats.global_start = high_resolution_clock::now();
	}

	Checker::~Checker()
	{
		if(bi_main_solver)
		{
			delete bi_main_solver;
			bi_main_solver = NULL;
		}
		if(bi_start_solver)
		{
			delete bi_start_solver;
			bi_start_solver = NULL;
		}
		if(bi_partial_solver)
		{
			delete bi_partial_solver;
			bi_partial_solver = NULL;
		}
	}


	namespace inv
	{
		/**
		 * @brief 
		 * Add the negation of this frame into the solver
		 * @param frame 
		 */
		void inv_solver_add_constraint_or(Frame& frame,int level, InvSolver* inv_solver_)
		{
			inv_solver_->add_constraint_or(frame,level);
		}

		/**
		 * @brief 
		 * Add the real states into this solver.
		 * @param frame 
		 */
		void inv_solver_add_constraint_and(Frame& frame,int level, InvSolver* inv_solver_)
		{
			inv_solver_->add_constraint_and(frame,level);
		}

		/**
		 * @brief pop the last lit of assumption, and negate it.
		 * 
		 */
		void inv_solver_release_constraint_and(InvSolver* inv_solver_, int level)
		{
			inv_solver_->release_constraint_and(level);
		}

		bool invariant_found_at(Fsequence &F_, const int frame_level,  InvSolver *inv_solver_, int minimal_update_level_)
		{
			if (frame_level <= minimal_update_level_)
			{
				inv_solver_add_constraint_or(F_[frame_level],frame_level, inv_solver_);
				return false;
			}
			inv_solver_add_constraint_and(F_[frame_level],frame_level, inv_solver_);
			bool res = !inv_solver_->solve_with_assumption();
			inv_solver_release_constraint_and(inv_solver_,frame_level);
			inv_solver_add_constraint_or(F_[frame_level],frame_level, inv_solver_);
			return res;
		}

		// -1 for not found.
		// >=0 for level
		int invariant_found(Model* model_, Fsequence &F_,int& minimal_update_level_, int frame_level)
		{
			if (frame_level == 0)
				return -1;
			int res = -1;
			InvSolver *new_inv_solver_ = new InvSolver(model_);
			for (int i = 0; i < frame_level; i++)
			{
				if (invariant_found_at(F_, i, new_inv_solver_,minimal_update_level_))
				{
					res = i;
					// delete frames after i, and the left F_ is the invariant
					// NOTE: this is temporarily blocked for testing incremental-enumratiing-start-solver.
					// while (F_.size() > i + 1)
					// {
					// 	F_.pop_back();
					// }
					break;
				}
			}
			delete new_inv_solver_;
			return res;
		}
	};

	// NOTE: if not updated, it return the same state all the time?
	State *Checker::enumerate_start_state(StartSolver *start_solver)
	{
		// partial state:
		#ifdef PARTIAL
		if(start_solver->solve_with_assumption())
		{
			Assignment ass= start_solver->get_model();
			ass.resize (model_->num_inputs() + model_->num_latches());
			State *partial_res = get_partial_state(ass,nullptr);
			clear_defer(partial_res);
			return partial_res;
		}
		#else
		if (start_solver->solve_with_assumption())
		{
			State *res = start_solver->create_new_state();
			clear_defer(res);
			return res;
		}
		#endif
		return NULL;
	}

	// This is used in sequence initialization
    bool Checker::immediate_check(State *from, int target, bool &res, Frame &O0)
    {
		// bi_main_solver.
		auto &solver = bi_main_solver; 
		// NOTE: init may not be already set.
		vector<int> latches = from->s();
		
		int last_max = 0;
		do
		{		
			if(solver->solve_with_assumption(latches,target))
			{
				// if sat. already find the cex.
				State *s = solver->get_state(true);// no need to shrink here
				clear_defer(s);
				which_prior()[s] = from;
				which_counter_start() = s;
				res = false;
				return true;
			}
			// NOTE: the last bit in uc is added in.
			Cube cu = solver->get_conflict(target); // filter 'bad'
			
			if (cu.empty())
			{
				// this means, ~p itself is bound to be UNSAT. No need to check.
				res = true;
				return true;
			}

			std::set<vector<int>> ucs;
			if(ucs.find(cu)!=ucs.end())
				break;
			// this time's max lit.
			// -2 : because we added a new bit to the end of uc.
			else if(abs(cu[cu.size()-2]) <= last_max)
				break;
			else
			{
				ucs.insert(cu);
				last_max = abs(cu[cu.size()-2]);
				O0.push_back(cu);				
			}
			auto unfresh = [&cu, &last_max](int x) {
        		return abs(x) > last_max;
    		};	
			std::stable_partition(latches.begin(), latches.end(), unfresh);
		}
		while(true);
        return false;
    }

	bool Checker::last_check(State* from,  Osequence* O)
	{
		// if(SO_map[State::negp_state] != O) 
		// 	return true;
		bool direction = !backward_first;
		if(direction)
		// in forward CAR, last target state is Init. Initial state is a concrete state, therefore it is bound to be reachable (within 0 steps). Here we only need to update the cex structure.
		{
			which_counter_start() = from;
			return true;
		}
		else
		// in backward car, we use uc0 instead of ~P to initialize Ob[0]. This makes it possible to return false, because uc0 is necessary but not essential.  
		{
			// check whether it's in it.
			bool res = bi_main_solver->solve_with_assumption(from->s(),bad_);
			if(res)
			{
				// OK. counter example is found.
				State *s = bi_main_solver->get_state(direction);
				clear_defer(s);
				which_prior()[s] = from;
				which_counter_start() = s;
				return true;
			}
		}
		return false;
	}

	void Checker::print_sat_query(MainSolver *solver, State* s, Osequence *o, int level, bool res, ostream& out)
	{
		static int sat_cnt = 0;
		out<<endl;
		out<<"[SAT] times: "<<++sat_cnt<<"\tlevel="<<level<<endl;
		//<<"\tflag="<<solver->bi_flag_of(o,level)<<endl;
		out<<"SAT solver clause size: "<<solver->size()<<endl;
		out<<"State: ";
		for(int i:s->s())
		out<<i<<", ";
		out<<endl;
		// cout<<"state: \n\tlatches: "<<s->latches()<<"\n\tinputs: "<<s->inputs()<<endl;
		// solver->print_clauses();
		solver->print_assumption();
		out<<"res: "<<(res ? "SAT" : "UNSAT")<<endl;
		out<<"-----End of printing-------"<<endl;
	}

	/**
	 * @brief 
	 * @pre s->s() is in abs-increasing order
	 * 
	 * @param s 
	 * @param frame_level 
	 * @param O 
	 * @param Otmp 
	 * @return true 
	 * @return false 
	 */
	bool Checker::blocked_in(const State* s, const int frame_level, Osequence *O, Frame & Otmp)
	{

		assert(frame_level >= 0);
		assert(frame_level <= O->size());
		Frame &frame = (frame_level < O->size()) ? (*O)[frame_level] : Otmp;
		for (auto &uc : frame)
		{
			if(s->imply(uc))
			{
				// if(frame_level == O->size()-1)
				// {
				// 	blocked_by[get_uc_id(uc)].push_back(s);
				// }
				return true;
			}	
		}
		return false;
	}

	// starting from frame_level, 
	int Checker::min_not_blocked(const State* s, const int min, const int max, Osequence *O, Frame & Otmp)
	{
		int start = min;
		while(start <= max)
		{
			if(!blocked_in(s,start,O,Otmp))
			{
				break;
			}
			start++;
		}
		return start;
	}
	

    void Checker::remove_s_from_U(State *s, Usequence &U)
    {
		int index;
		for(index = 0; index < U.size();++index)
		{
			auto it = std::find(U[index].begin(),U[index].end(), s);
			if(it != U[index].end())
			{
				break;
			}
		}
		auto prior = which_prior();
		State *p = s;
		while(p && index < U.size())
		{
			auto& subvec=U[index];
			subvec.erase(std::remove(subvec.begin(), subvec.end(), p), subvec.end());
			p = prior[p];
			index++;
		}



			
    }
	
}
