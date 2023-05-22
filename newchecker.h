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


#ifndef NEWCHECKER_H
#define NEWCHECKER_H

#include "data_structure.h"
#include "invsolver.h"
#include "startsolver.h"
#include "newmainsolver.h"
#include "newpartialsolver.h"
#include "model.h"
#include <assert.h>
#include "utility.h"
#include "statistics.h"
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <fstream>
#include <map>

namespace car
{
	extern Statistics stats; // defined in main.cpp
	extern bool verbose_;	 // defined in main.cpp
	extern Model *model;	 // defined in main.cpp
	extern int storage_id;
	class Checker;
	extern Checker ch;

	// using Osequence = std::vector<Frame>;
	// using Usequence = std::vector<std::vector<State*> >;


	class Checker
	{
	public:
		Checker(Model *model,std::ostream &out,std::ofstream &trail_out,std::ofstream &dot_out,std::ofstream &dive_out, bool enable_dive, bool forward = true, bool propage = false, bool evidence = false, int index_to_check = 0, bool enable_bi=false);
		~Checker();

		// entrance for bi-car checking
		bool bi_check();


	private:
		Model *model_; 
		bool evidence_;
		int bad_;
		// to get state from ~p
		StartSolver *bi_start_solver;
	
	private:
		bool backward_first = true;
		bool enable_bi_check = false;

		// mapping from state to its corresponding o sequence.
		hash_map<const State*,Osequence*> SO_map;
		// the states blocked.
		hash_set<State *> blocked_states;
		// the main solver shared.
		MainSolver *bi_main_solver;
		// the partial solver shared.
		PartialSolver *bi_partial_solver;
		// count of blocked states.
		int blocked_count=0;

		// the map from O sequence to its minimal_level
		hash_map<const Osequence*, int> fresh_levels;
		hash_map<const Osequence*, std::vector<Trie *>*> OT_map;
		Usequence Uf, Ub; //Uf[0] is not explicitly constructed 
		Osequence Onp, OI;
		
	public:
		inline Usequence& which_U()
		{
			return backward_first ? Ub : Uf;
		}
		
		inline Usequence& other_U()
		{
			return backward_first ? Uf : Ub;
		}
	public:

		// std::vector<Trie*> Tnp, TI;
		// inline std::vector<Trie*>& which_trie(Osequence *o)
		// {
		// 	assert(OT_map.count(o));
		// 	return *(OT_map[o]);
		// } 

		// record the prior state in this trail. This will be used in counter example printing.
		std::map<State *, State*> prior_in_trail_f;
		std::map<State *, State*> prior_in_trail_b;
		inline std::map<State *, State*>& which_prior()
		{
			return backward_first ? prior_in_trail_b : prior_in_trail_f;
		}

		State* counter_start_f = nullptr;
		State* counter_start_b = nullptr;
		inline State*& which_counter_start()
		{
			return backward_first ? counter_start_b : counter_start_f;
		}
		inline State*& other_counter_start()
		{
			return backward_first ? counter_start_f : counter_start_b;
		}
		std::ostream &out;
		std::ostream &trail_out;
		std::ofstream &dot_out;
		std::ofstream &dive_out;
		bool enable_dive;

	private:
		// clear duties
		
		// the states to be cleared
		std::unordered_set<State*> clear_duties;
		// add into clear duty, waiting for clear in the end.
		inline void clear_defer(State* s){clear_duties.insert(s);};
		// the main solver to be cleared
		hash_set<MainSolver*> clear_duties_mainsolver;
		// add into clear duty, waiting for clear in the end.
		inline void clear_defer(MainSolver* s){clear_duties_mainsolver.insert(s);};

	private:
	// #ifdef RANDOM_PICK 
		std::vector<std::pair<State *, int>> Uset;
	// #endif // RANDOM_PICK

	// #ifdef PARTITION
		std::map<int, std::vector<const State*>> blocked_by; 
		int max_uc_id = 0;
		std::map<std::vector<int>, int> uc_ids;
		inline int get_uc_id(const std::vector<int> &v)
		{
			if (uc_ids.count(v) == 0)
				uc_ids[v] = ++max_uc_id;
			return uc_ids[v];
		}
		std::map<int,int> uc_reduce;
	// #endif

	private:
		// bi-serach method section

		/**
		 * @brief Check for immediate safe or unsafe
		 * 
		 * @param out where to print
		 * @param res the check result
		 * @return true : Bad == True / False
		 * @return false 
		 */
		bool trivial_check(bool& res);

		/**
		 * @brief The main procedure of Bi-CAR
		 * 
		 * @return true : Safe
		 * @return false : Unsafe
		 */
		bool bi_car();

		bool car_inv();

		/**
		 * @brief Searching procedure of bi-car. Enumerate states in U and use the Osequence as the guide.
		 * 
		 * @param U the U sequence to enumerate. 
		 * @param O the O sequence with a singleton (a state that is picked from the U sequence) making up its level 0.
		 * @param forward whether the Transformation relationship should be used forward or backward
		 * @return true : successfully reached O[0]. which means a cex is found.
		 * @return false : all states in present U has been checked. No cex is found.
		 */
		bool bi_try_satisfy(Usequence &U, Osequence *O, bool forward, bool& safe_reported);
		bool inv_try_satisfy(State* missionary, Usequence &U, Osequence *O, int level,bool& safe_reported, InvSolver * inv_solver);
		bool common_try_satisfy(Usequence &U, Osequence *O, bool forward, bool& safe_reported);
		
		/**
		 * @brief Check whether there exists an invariant in this sequence. Which means the center state of this Osequence is not reachable, and should be later blocked
		 * 
		 * @param O the sequence to be checked
		 * @return true : invariant is found
		 * @return false : no invariant is found
		 */
		bool bi_invariant_found(Osequence *O);

		/**
		 * @brief Create a O with given state object as its level 0.
		 * 
		 * @param s 
		 * @return Osequence& 
		 */
		Osequence* create_O_with_state(State *s);


		/**
		 * @brief iteratively pick state from the given sequence. 
		 * @return State*  
		 * 	then this state will be marked as visited at this round.
		 * @return nullptr every state has been tried.
		 *  then all the markers will be cleaned. Therefore, this method can be reused for another round.
		 * @post Anytime it ends the iterative round earlier(before nullptr), it must has found the counter example in bi_try_satisfy. Therefore, there will be no picking in the future. 
		 */
		State *pick_state(Usequence &);
		State *pick_state_random(Usequence &);
		State *pick_state(Usequence &, int &level);
		State *pick_state_random(Usequence &, int &level);
		State *pick_state_descendant_inc(Usequence &, int &level);
		State *pick_state_descendant_dec(Usequence &, int &level);
		State *pick_state_children_inc(Usequence &, int &level);
		State *pick_state_children_dec(Usequence &, int &level);
		State *pick_state_descendant_inc_anc(Usequence &, int &level);
		State *pick_state_descendant_dec_anc(Usequence &, int &level);
		State *pick_state_children_inc_anc(Usequence &, int &level);
		State *pick_state_children_dec_anc(Usequence &, int &level);
		State *pick_state_descendant_dec_heavy(Usequence &, int &level);
		State *pick_state_descendant_inc_heavy(Usequence &, int &level);
		State *pick_state_partition_inc(Usequence &, int &level);
		State *pick_state_partition_dec(Usequence &, int &level);
		State *pick_state_restart(Usequence &, int &level);
		State *pick_state_oneshot(Usequence &, int &level);
		
		State* get_partial_state(Assignment &s, const State* prior_state);

		void reorder_seq(std::vector<int> & vec, int state_level);

		/**
		 * @brief : each level contains freq[i].
		 * 
		 */
		std::vector<std::unordered_map<int, int>> Fl;
		void reorder_according_to_Fl(std::vector<int>& vec, int level);

		/**
		 * @brief print evidence to out.
		 * 
		 * @param out 
		 */
		void bi_print_evidence() const;



		/**
		 * @brief Get the solution from SAT solver.
		 * 
		 * @return State* : the state representing the solution, which is to be added to the U sequence.
		 */
		State* getModel(MainSolver*);
		State* getModel(MainSolver*,State*);
		
		/**
		 * @brief Update U sequence, and push into cex vector
		 * 
		 * @param level 
		 * @return whether it is already seen at a lower level.
		 */
		bool update_U(Usequence&, State*, int level, State* prior_in_trail);


		/**
		 * @brief SAT_Assume(assum, clauses)
		 * 
		 * @return true 
		 * @return false 
		 */
		bool sat_assume(MainSolver*, Osequence *O, State*, int, const Frame& Otmp);
		bool sat_assume_common(MainSolver*, Osequence *O, State*, int, const Frame& Otmp, const std::vector<State*>& state_of_level, int & how_many);

		/**
		 * @brief Interface for cleaning.
		 * 
		 */
		void bi_clean();

		/**
		 * @brief When Os finds an invariant, clear Os and erase s from Os.
		 * 
		 * @param s 
		 * @param Os 
		 */
		void clear_Os(State* s, Osequence *Os);

		/**
		 * @brief Block this state in main solver forever.
		 * Because it is blocked forever, there is no need for a flag.
		 */
		void block_state_forever(const State*);

		/**
		 * @brief first create a clause with target uc and flag of the corresponding O[dst], then add this clause to main solver or start solver, depending on the level.
		 * 
		 * @param uc 
		 * @param O
		 * @param dst_level
		 */
		void add_uc_to_solver(Cube& uc, Osequence *O, int dst_level,Frame& Otmp);

		void update_O(Osequence *O, int dst_level, Frame &Otmp, bool& safe_reported);

		/**
		 * @brief init special sequences: Uf, Ub, Oi, Onp
		 */
		bool bi_init_sequences(bool& res);
		bool inv_init_sequences(bool& res);

		/**
		 * @brief Use start solver to get a state. Usually in ~p.
		 * 
		 * @param start_solver 
		 * @return State* 
		 */
		State* enumerate_start_state(StartSolver *start_solver);

		/**
		 * @brief Check SAT_ASSUME(I, ~P).
		 * 
		 * to generate uc from init
		 * ~p should be in ~uc
		 * use uc to initialize O[0] is suitable. 
		 * 
		 */
		bool immediate_check(State *from, int target, bool &res, Frame &O0);

		bool last_check(State* from, Osequence* O);

		void print_sat_query(MainSolver *solver, State* s, Osequence *o, int level, bool res, std::ostream& out);

		bool blocked_in(const State* s, const int frame_level, Osequence *O, Frame & Otmp);

		int min_not_blocked(const State*s, const int min, const int max, Osequence *O, Frame &Otmp);

		// int get_new_level(Osequence *O, const State *s, const int level);

		void print_flags(std::ostream &out);

		void remove_s_from_U(State*s, Usequence&U);

		bool bi_invariant_found_at(Osequence& O, int check_level,int minimal_update_level, InvSolver *inv_solver);

	private:
		// simplify U
		void simplify_U_children();
		
		void simplify_U_descendant();


	public:
		void bi_draw_graph();
		void print_U_shape();
		std::vector<int> spliter;
		std::set<int> blocked_ids;
		std::vector<int> blocked_counter_array;
		int direct_blocked_counter = 0;
		void print_U_sequnece(const Usequence & U, std::ostream & out_stream);

	private:
		// intersection
		// if we want to analyze
		// std::unordered_map<int,int> freq;

		// rotation
		// corresponding to O[i]
		std::vector<Cube> rotates;
		// corresponding to Otmp
		Cube rotate;

		#ifdef RECENT
		std::vector<Cube> recent;
		#endif // RECENT

		#ifdef MORE_INTER
		std::vector<Cube> asmsubs;
		Cube asmsub;
		#endif // MORE_INTER

	public:
		// this section is for drawing diving-graph in O-level.
		int dive_counter = 0;
		int dive_maxx = 0;
		State* dive_last_state = nullptr;
		std::map<int,std::string> colors = {
			// black
			{0,"#000000"},
			// red
			{1,"#f00003"},
			// green
			{2,"#008000"},
			// purple
			{3,"#800080"}
		};
		inline void dive_draw_wedge(State* from, int from_level, State*to, int to_level, int color, int wedge_label)
		{
			dive_out<<"// "<<from->id<<"/"<<from_level<<" , "<<int(to? to->id : -1)<<"/"<<to_level<<std::endl;
			if(from == to)
			// backtrack
			{
				if(from != dive_last_state)
				{
					// some state is picked, and fails at once
					if(dive_counter!=1)
						dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter<<"[style=dotted] ;"<<std::endl;
					dive_maxx++;
					dive_draw_vextex(from,from_level, dive_maxx, false);
				}
				dive_maxx++;
				dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter;
				dive_out<<"[style=dashed";
				dive_out<<", color=\""<<colors[color]<<"\"";
				if(wedge_label >= 1)
					dive_out<<", label=\""<<wedge_label<<"\", fontcolor=\""<<colors[color]<<"\"";
				dive_out<<"] ;"<<std::endl;
				dive_draw_vextex(to,to_level, dive_maxx, false);
				dive_last_state = to;
			}
			else if(!to)
			// from is left out
			{
				// if(from != dive_last_state)
				// {
				// 	dive_maxx++;
				// 	dive_counter++;
				// 	dive_draw_vextex(from,from_level, dive_maxx, false);
				// }
				// else
				// {
				// 	dive_draw_vextex(from,from_level, dive_maxx, true);
				// 	dive_counter++;
				// }

				if(from != dive_last_state)
				{
					if(dive_counter!=1)
						dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter<<"[style=dotted] ;"<<std::endl;
					dive_maxx++;
					// dive_counter++;
					dive_draw_vextex(from,from_level, dive_maxx, false);
				}
				dive_maxx++;
				
				dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter<<"[style=dotted";
				dive_out<<", color=\""<<colors[color]<<"\"";
				if(wedge_label >= 1)
					dive_out<<", label=\""<<wedge_label<<"\", fontcolor=\""<<colors[color]<<"\"";
				dive_out<<"] ;"<<std::endl;
				
				dive_out<< "s" << dive_counter << "[label=OUT, style=dotted,  pos=\""<<dive_maxx<< ","<<from_level+2<<"!\"]"<<std::endl;

				

				dive_last_state=nullptr;
			}
			else 
			{
				// continue to try
				if(from != dive_last_state)
				{
					if(dive_counter!=1)
						dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter<<"[style=dotted] ;"<<std::endl;
					dive_maxx++;
					dive_draw_vextex(from,from_level, dive_maxx, false);
				}
				// else : another state to try
				dive_out <<"s"<< dive_counter++ <<" -> "<< "s"<<dive_counter;
					dive_out<<"[color=\""<<colors[color]<<"\"";
				if(wedge_label >= 1)
					dive_out<<", label=\""<<wedge_label<<"\", fontcolor=\""<<colors[color]<<"\"";
				dive_out<<"] ;"<<std::endl;
				dive_draw_vextex(to,to_level, dive_maxx, false);
				dive_last_state = to;
			}


		}

		inline void dive_draw_head()
		{
			dive_out<<"digraph G {"<<std::endl;
			dive_counter = 1;
			dive_out<<
"\
tl11[style=\"invis\", pos=\"0,8!\"]; \n\
tl11 -> tl12[style=\"invis\",label=\"legend\"];\n\
tl12[style=\"invis\", pos=\"3,8!\"]; \n\
\n\
tl21[style=\"invis\", pos=\"0,7!\"]; \n\
tl21 -> tl22[color=\""<<colors[0].c_str()<<"\",label=\"basic\"];\n\
tl22[style=\"invis\", pos=\"3,7!\"]; \n\
\n\
tl31[style=\"invis\", pos=\"0,6!\"]; \n\
tl31 -> tl32[color=\""<<colors[1].c_str()<<"\",label=\"tried before\"];\n\
tl32[style=\"invis\", pos=\"3,6!\"]; \n\
\n\
tl41[style=\"invis\", pos=\"0,5!\"]; \n\
tl41 -> tl42[color=\""<<colors[2].c_str()<<"\",label=\"same O frame\"];\n\
tl42[style=\"invis\", pos=\"3,5!\"]; \n\
\n\
tl51[style=\"invis\", pos=\"0,4!\"]; \n\
tl51 -> tl52[color=\""<<colors[3].c_str()<<"\",label=\"same state to this level\"];\n\
tl52[style=\"invis\", pos=\"3,4!\"]; \n\
"<<std::endl;
		}

		inline void dive_mark_bad()
		{
			dive_out<<"s"<<dive_counter<<"[label = BAD, color=blue, pos=\"" <<dive_maxx<<",-1!\"]"<<std::endl;
		}

		inline void dive_draw_tail()
		{
			dive_out<<"}"<<std::endl;
		}

		inline void dive_draw_vextex(State*s, int level,int posx,bool dash)
		{
			dive_out<< "s" << dive_counter << "[label="<<s->id<<", pos=\""<<posx<< ","<<level<<"!\"";
			if(dash)
				dive_out<<",style=dashed] ;"<<std::endl;
			else
				dive_out<<"]; "<<std::endl;
		}

	public:

		/**
		 * @brief reorder asgn, by intersection with s->s(). 
		 * 		  asgn-> (asgn ∩ s, asgn - s)
		 * @param asgn The asgn to reorder
		 * @param s the state to intersect with
		 * @param max_index if max_index < asgn's max index, it means only a part of it is the asgn.  
		 * @return int : the new max_index 
		 */

		int reorder_intersect_one(Cube& asgn, const State* s, int max_index);
		void reorder_intersect(Cube& asgn, const std::vector<State*>& frame, std::vector<int>& pos);
		void reorder_intersect_unittest(State*s);
		int reorder_intersect_how_many(const Cube& asgn,const Cube& res,std::vector<int>& pos);
		void reorder_intersect_how_many_unittest(State *s);
	public:
		std::vector<std::pair<int,int>> longest_uc_index;
		std::vector<std::pair<int,int>> shortest_uc_index;
	
	private:
		#ifdef PP_LIMIT_TIME
		bool isppnow=true;
		#else
		bool isppnow = false;
		#endif
		bool ppstoped=false;
		#if defined(INTER) && !defined(PP_LIMIT_TIME) // a fixed inter cnt.
			#ifdef INTER_CNT
			int inter_cnt = INTER_CNT;
			#else
			int inter_cnt = 1;
			#endif // INTER_CNT
		#else
		int inter_cnt = 0;
		#endif
		#if defined(ROTATE) && !defined(PP_LIMIT_TIME)
		bool rotate_enabled = true;
		#else
		bool rotate_enabled = false;
		#endif
		static int sat_counter;
		static clock_high sat_timer;
	private:
		std::vector<Osequence> uc_knowledge;
		Osequence cooked_knowledge;
		void cook();
		void cook_light();
		void pick_next_pp();
	public:
		inline void set_inter_cnt(int cnt){inter_cnt = cnt;}
		inline int get_inter_cnt(){return inter_cnt;}
		inline void set_rotate(bool rtt){rotate_enabled = rtt;}
		inline int get_rotate(){return rotate_enabled;}
		
	public:
		// whether this is a fresh O frame, which means uc in it has just been updated and never does a SATAssume take place.
		std::vector<bool> O_level_fresh;
		std::vector<int> O_level_fresh_counter;
		// the last state id to visit this level. 
		std::vector<int> O_level_repeat;
		std::vector<int> O_level_repeat_counter;

	};


	namespace inv
	{
		void inv_solver_add_constraint_or(Frame& frame, int level, InvSolver* inv_solver_);

		void inv_solver_add_constraint_and(Frame& frame, int level, InvSolver* inv_solver_);

		void inv_solver_release_constraint_and(InvSolver* inv_solver_, int level);

		bool invariant_found_at(Fsequence &F_, const int frame_level,  InvSolver *inv_solver_, int minimal_update_level_);
	}

}


#endif
