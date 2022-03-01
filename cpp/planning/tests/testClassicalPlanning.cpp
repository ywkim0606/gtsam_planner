/*
 * testClassicalPlanning.cpp
 * @brief develop code for Classical Planning solver
 * @date Nov 3, 2021
 * @author Yoonwoo Kim
 */

#include <cpp/planning/CSP.h>

#include <boost/assign/std/map.hpp>

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

#include <fstream>
#include <iostream>
#include <numeric>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/assign.hpp>
#include <boost/algorithm/string/join.hpp>
#include <algorithm>
#include <valarray>     // std::valarray, std::slice

using namespace boost::assign;
using namespace std;
using namespace gtsam;

vector<DiscreteKey> generate_state(int t) {
  size_t nrBool = 2;
  size_t nrloc1 = 5;
  size_t nrloc2 = 4;
  size_t nrloc3 = 3;
  DiscreteKey cl1(10*t, nrBool), cl2(10*t+1, nrBool), cl3(10*t+2, nrBool),
    clA(10*t+3, nrBool), clB(10*t+4, nrBool), clC(10*t+5, nrBool), on1(10*t+6, nrloc1),
    on2(10*t+7, nrloc2), on3(10*t+8, nrloc3);
  vector<DiscreteKey> dkeys {cl1, cl2, cl3, clA, clB, clC, on1, on2, on3};
  return dkeys;
}

DiscreteKey generate_action(int t) {
  size_t nrMoves = 50;
  DiscreteKey move(10*t+9, nrMoves);
  return move;
}

vector<string> generate_moves() {
  // generate all possible moves
  vector<string> disks = {"1", "2", "3"};
  vector<string> loc1 = {"2", "3", "A", "B", "C"};
  vector<string> loc2 = {"3", "A", "B", "C"};
  vector<string> loc3 = {"A", "B", "C"};
  vector<string> moves;
  for (string d : disks) {
    if (d == "1") {
      for (string from: loc1) {
        for (string to: loc1) {
          moves.push_back(d+from+to);
        }
      }
    } else if (d == "2") {
      for (string from: loc2) {
        for (string to: loc2) {
          moves.push_back(d+from+to);
        }
      }
    } else if (d == "3") {
      for (string from: loc3) {
        for (string to: loc3) {
          moves.push_back(d+from+to);
        }
      }
    }
  }
  return moves;
}

string generate_string(vector<string> input_list, string target) {
  map<string, string> string_map;
  for (string element: input_list) {
    string_map.insert(pair<string, string>(element, "0"));
  }
  string_map.at(target) = "1";
  vector<string> o;
  boost::copy(string_map | boost::adaptors::map_values, std::back_inserter(o));
  return boost::algorithm::join(o, " ");
}

string generate_bin_string(bool input){
  return input ? "0 1" : "1 0";
}

int findIndex(const vector<string> &arr, string item) {
  auto ret = std::find(arr.begin(), arr.end(), item);
  if (ret != arr.end())
      return distance(arr.begin(), ret);
  return -1;
}

vector<DecisionTreeFactor> generate_factor(const vector<DiscreteKey> state_t,
  const DiscreteKey move_t, const vector<DiscreteKey> state_tp, const vector<string> moves) {
    vector<string> disks = {"1", "2", "3"};
    vector<string> loc1 = {"2", "3", "A", "B", "C"};
    vector<string> loc2 = {"3", "A", "B", "C"};
    vector<string> loc3 = {"A", "B", "C"};
    map<string, vector<string>> loc_map;
    loc_map["1"] = loc1;
    loc_map["2"] = loc2;
    loc_map["3"] = loc3;

    vector<string> clear_list = {"1", "2", "3", "A", "B", "C"};
    vector<DiscreteKey> clear_t = vector<DiscreteKey>(state_t.begin(), state_t.begin()+6);
    vector<DiscreteKey> on_t = vector<DiscreteKey>(state_t.begin()+6, state_t.end());
    vector<DiscreteKey> clear_tp = vector<DiscreteKey>(state_tp.begin(), state_tp.begin()+6);
    vector<DiscreteKey> on_tp = vector<DiscreteKey>(state_tp.begin()+6, state_tp.end());

    vector<DecisionTreeFactor> factors;
    DecisionTreeFactor factor;

    for (string move: moves) {
      string disk = string(1, move.at(0));
      string from_loc = string(1, move.at(1));
      string to_loc = string(1, move.at(2));
      if (from_loc == to_loc) {
        // precondition:
        // disk should be clear to move
        DecisionTreeFactor f1(clear_t.at(findIndex(clear_list, disk)), generate_bin_string(true));
        // disk has to be on the from_loc
        DecisionTreeFactor f2(on_t.at(findIndex(disks, disk)), generate_string(loc_map[disk], from_loc));
        // to_loc should not be clear
        DecisionTreeFactor f3(clear_t.at(findIndex(clear_list, to_loc)), generate_bin_string(true));
        // action
        DecisionTreeFactor f4(move_t, generate_string(moves, move));
        // effect:
        // disk has to be on the to_loc
        DecisionTreeFactor f5(on_tp.at(findIndex(disks, disk)), generate_string(loc_map[disk], to_loc));
        // to_loc is not clear
        DecisionTreeFactor f6(clear_tp.at(findIndex(clear_list, to_loc)), generate_bin_string(false));
        // from loc is not clear
        DecisionTreeFactor f7(clear_tp.at(findIndex(clear_list, from_loc)), generate_bin_string(false));
        factor = f1 * f2 * f3 * f4 * f5 * f6 * f7;
      }
      else {
        // precondition:
        // disk should be clear to move
        DecisionTreeFactor f1(clear_t.at(findIndex(clear_list, disk)), generate_bin_string(true));
        // cout << clear_t.at(findIndex(clear_list, disk)).first << endl;
        // disk has to be on the from_loc
        DecisionTreeFactor f2(on_t.at(findIndex(disks, disk)), generate_string(loc_map[disk], from_loc));
        // to_loc should not be clear
        DecisionTreeFactor f3(clear_t.at(findIndex(clear_list, to_loc)), generate_bin_string(true));
        // action
        DecisionTreeFactor f4(move_t, generate_string(moves, move));
        // effect:
        // disk has to be on the to_loc
        DecisionTreeFactor f5(on_tp.at(findIndex(disks, disk)), generate_string(loc_map[disk], to_loc));
        // to_loc is not clear
        DecisionTreeFactor f6(clear_tp.at(findIndex(clear_list, to_loc)), generate_bin_string(false));
        // from loc is clear
        DecisionTreeFactor f7(clear_tp.at(findIndex(clear_list, from_loc)), generate_bin_string(true));
        factor = f1 * f2 * f3 * f4 * f5 * f6 * f7;
      }
      factors.push_back(factor);
    }
    return factors;
}

// A class that encodes classical planning as a CSP problem
CSP planner(int k, vector<size_t> init, vector<size_t> goal) {
  if (k < 1) {
    throw runtime_error("Plan length should be at least one");
  }

  vector<vector<DiscreteKey>> states;
  vector<DiscreteKey> actions;
  vector<vector<DecisionTreeFactor>> factors;
  CSP csp;

  for (int i = 0; i < k-1; i++) {
    vector<DiscreteKey> state = generate_state(i);
    // if (i == 0) {
    //   // add unary constraints for initial state
    //   for (size_t j = 0; j < init.size(); j++) {
    //     csp.addSingleValue(state.at(j), init.at(j));
    //   }
    // }
    DiscreteKey action = generate_action(i);
    states.push_back(state);
    actions.push_back(action);
  }
  vector<DiscreteKey> last_state = generate_state(k-1);
  // for (size_t i = 0; i < goal.size(); i++) {
  //   csp.addSingleValue(last_state.at(i), goal.at(i));
  // }
  states.push_back(last_state);
  vector<string> moves = generate_moves();
  for (size_t i = 0; i < states.size()-1; i++) {
    vector<DecisionTreeFactor> factor = generate_factor(states.at(i),
      actions.at(i), states.at(i+1), moves);
    factors.push_back(factor); 
  }
  for (vector<DecisionTreeFactor> factor: factors) {
    for (DecisionTreeFactor f_t: factor) {
      csp.push_back(f_t);
    }
  }
  return csp;
}

// int plan_length(int max_length, vector<size_t> initial_state,
//   vector<size_t> goal_state, CSP csp_plan) {
//   for (int i = 3; i < max_length; i++) {
//     auto mpe = csp_plan.optimize();
//     int  output = csp_plan(mpe);
//     if (output == 1) {
//       return i;
//     }
//     cout << output << endl;
//     CSP csp_plan = planner(i, initial_state, goal_state);
//   }
//   return 0;
// }
  

/* ************************************************************************* */
TEST(CSP, Multiply) {
  // Create keys for On1, On2, On3, Cl1, Cl2, Cl3, ClA, ClB, ClC
  size_t nrDisks = 3;           // number of disks
  size_t nrTowers = 3;          // number of towers
  size_t nrBool = 2;            // true/false

  DiscreteKey Cl1_0(0, nrBool), ClB_0(1, nrBool), On1_0(2, nrDisks+nrTowers),
    Cl1_1(3, nrBool), ClB_1(4, nrBool), On1_1(5, nrDisks+nrTowers);
  
  vector<DiscreteKey> dkeys{Cl1_0, ClB_0, On1_0, Cl1_1, ClB_1, On1_1};
  
  // preconditions:
  // 1. disk should be clear to move 
  DecisionTreeFactor fCl1_0(Cl1_0, "0 1");
  // 2. loc2 should be clear
  DecisionTreeFactor fClB_0(ClB_0, "0 1");
  // 3. disk should be on loc1
  DecisionTreeFactor fOn1_0(On1_0, "0 1 0 0 0 0");
  // effects:
  // 1. disk should still be clear to move
  DecisionTreeFactor fCl1_1(Cl1_1, "0 1");
  // 2. loc2 should not be clear
  DecisionTreeFactor fClB_1(ClB_1, "1 0");
  // 3. disk should be on loc2
  DecisionTreeFactor fOn1_1(On1_1, "0 0 0 0 1 0");

  DecisionTreeFactor expected(Cl1_0 & ClB_0, "0 0 0 1");
  DecisionTreeFactor actual = fCl1_0 * fClB_0;
  CHECK(assert_equal(expected, actual));

  DecisionTreeFactor combined1 = fCl1_0 * fClB_0 * fOn1_0;
  DecisionTreeFactor combined2 = fCl1_1 * fClB_1 * fOn1_1;
  DecisionTreeFactor combined3 = combined1 * combined2;

  DiscreteValues values;
  values[0] = 1; // true
  values[1] = 1; // true
  values[2] = 1; // 2
  values[3] = 1; // true
  values[4] = 0; // false
  values[5] = 4; // B

  EXPECT_DOUBLES_EQUAL(1, combined1(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(1, combined2(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(1, combined3(values), 1e-9);
}

/* ************************************************************************* */
TEST(CSP, GenerateState) {
  vector<DiscreteKey> state = generate_state(0);
  EXPECT_DOUBLES_EQUAL(9, state.size(), 1e-9);
}

TEST(CSP, GenerateAction) {
  DiscreteKey move = generate_action(0);
  EXPECT_DOUBLES_EQUAL(50, move.second, 1e-9);
}

TEST(CSP, GenerateMoves) {
  vector<string> moves = generate_moves();
  string expected_first = "122";
  string expected_last = "3CC";
  EXPECT_DOUBLES_EQUAL(50, moves.size(), 1e-9);
  EXPECT(expected_first == moves.front());
  EXPECT(expected_last == moves.back());
}

TEST(CSP, GenerateString) {
  vector<string> example = {"A", "B", "C", "D", "E"};
  string target = "D";
  string generated_string = generate_string(example, target);
  string expected = "0 0 0 1 0";
  EXPECT(expected == generated_string);
}

TEST(CSP, GenerateBinString) {
  string generated_true = generate_bin_string(true);
  string generated_false = generate_bin_string(false);
  string true_expected = "0 1";
  string false_expected = "1 0";
  EXPECT(true_expected == generated_true);
  EXPECT(false_expected == generated_false);
}

TEST(CSP, FindIndex) {
  vector<string> example = {"A", "B", "C", "D", "E"};
  string target = "D";
  int generated_index = findIndex(example, target);
  int expected = 3;
  EXPECT(expected == generated_index);
}

TEST(CSP, GenerateFactor) {
  vector<DiscreteKey> state0 = generate_state(0);
  DiscreteKey move0 = generate_action(0);
  vector<DiscreteKey> state1 = generate_state(1);
  vector<string> possible_moves = generate_moves();
  vector<DecisionTreeFactor> factors = generate_factor(state0, move0, state1, possible_moves);
  // 50 factors because 50 possible moves
  EXPECT_DOUBLES_EQUAL(50, factors.size(), 1e-9);
  
  DecisionTreeFactor f1 = factors.at(3);
  DiscreteValues values;
  values[0] = 1; // cl1
  values[1] = 0; // cl2
  values[2] = 0; // cl3
  values[3] = 0; // clA
  values[4] = 1; // clB
  values[5] = 1; // clC
  values[6] = 0; // on1 = 2
  values[7] = 0; // on2 = 3
  values[8] = 0; // on3 = A
  values[9] = 3; // move = 12B
  values[10] = 1; // cl1
  values[11] = 1; // cl2
  values[12] = 0; // cl3
  values[13] = 0; // clA
  values[14] = 0; // clB
  values[15] = 1; // clC
  values[16] = 3; // on1 = B {2, 3, A, B, C} 
  values[17] = 0; // on2 = 3
  values[18] = 0; // on3 = A

  EXPECT_DOUBLES_EQUAL(1, f1(values), 1e-9);
}
TEST(CSP, AddFactor) {
  CSP csp_plan;
  vector<DiscreteKey> state0 = generate_state(0);
  DiscreteKey move0 = generate_action(0);
  vector<DiscreteKey> state1 = generate_state(1);
  vector<string> possible_moves = generate_moves();
  vector<DecisionTreeFactor> factors = generate_factor(state0, move0, state1, possible_moves);
  csp_plan.push_back(factors[3]);
  
  DiscreteValues values;
  values[0] = 1; // cl1
  values[1] = 0; // cl2
  values[2] = 0; // cl3
  values[3] = 0; // clA
  values[4] = 1; // clB
  values[5] = 1; // clC
  values[6] = 0; // on1 = 2
  values[7] = 0; // on2 = 3
  values[8] = 0; // on3 = A
  values[9] = 3; // move = 12B
  values[10] = 1; // cl1
  values[11] = 1; // cl2
  values[12] = 0; // cl3
  values[13] = 0; // clA
  values[14] = 0; // clB
  values[15] = 1; // clC
  values[16] = 3; // on1 = B {2, 3, A, B, C} 
  values[17] = 0; // on2 = 3
  values[18] = 0; // on3 = A

  EXPECT_DOUBLES_EQUAL(1, csp_plan(values), 1e-9);
}
TEST(CSP, AddSingleValue) {
  // cl1, cl2, cl3, clA, clB, clC, on1, on2, on3
  //  1    0    0    0    1    1   2(0) 3(0) A(0)
  vector<size_t> initial_state = {1, 0 ,0, 0, 1, 1, 0, 0, 0};
  //  1    1    0    0    0    1   B(3) 3(0) A(0)
  vector<size_t> goal_state = {1, 1 ,0, 0, 0, 1, 3, 0, 0};
  CSP csp;
  vector<DiscreteKey> state0 = generate_state(0);
  DiscreteKey move0 = generate_action(0);
  vector<DiscreteKey> state1 = generate_state(1);
  vector<string> possible_moves = generate_moves();
  vector<DecisionTreeFactor> factors = generate_factor(state0, move0, state1, possible_moves);
  
  for (size_t i = 0; i < initial_state.size(); i++) {
    csp.addSingleValue(state0.at(i), initial_state.at(i));
  }

  for (size_t j = 0; j < goal_state.size(); j++) {
    csp.addSingleValue(state1.at(j), goal_state.at(j));
  }
  
  csp.push_back(factors[3]);

  auto mpe = csp.optimize();
  GTSAM_PRINT(mpe);
  EXPECT_DOUBLES_EQUAL(1, csp(mpe), 1e-9);

  // should only have to figure out move == 12B
  // CSP csp_plan = planner(2, initial_state, goal_state);

  // auto mpe = csp_plan.optimize();
  // EXPECT_DOUBLES_EQUAL(1, csp_plan(mpe), 1e-9);
  // GTSAM_PRINT(mpe);
}


  // int length = plan_length(200, initial_state, goal_state, csp_plan);

  // cout << length << endl;

  // preconditions:
  // disk has to be clear
  
  // DecisionTreeFactor expected1(clear0[findIndex(clear_list, disk)], "0 1");
  // EXPECT(assert_equal(f1, expected1));
  // disk has to be on the from_loc
  
  // DecisionTreeFactor expected2(on0[findIndex(disks, disk)], "1 0 0 0 0");
  // EXPECT_DOUBLES_EQUAL(0, findIndex(disks, disk), 1e-9);
  // EXPECT(assert_equal(f2, expected2));
  // to_loc has to be clear
  // DecisionTreeFactor expected3(clear0[findIndex(clear_list, to_loc)], "0 1");
  // EXPECT(assert_equal(f3, expected3));
  // action
  // disk has to be on the to_loc
  // DecisionTreeFactor expected5(on1[findIndex(disks, disk)], "0 0 0 1 0");
  // EXPECT(assert_equal(f5, expected5));
  // to_loc is not clear
  
  // DecisionTreeFactor expected6(clear1[findIndex(clear_list, to_loc)], "1 0");
  // EXPECT(assert_equal(f6, expected6));
  // from_loc is clear
  
  // DecisionTreeFactor expected7(clear1[findIndex(clear_list, from_loc)], "0 1");
  // EXPECT(assert_equal(f7, expected7));

  // DecisionTreeFactor combined = f1 * f2 * f3 * f4 * f5 * f6 * f7;
//   combined.dot("move12b", DefaultKeyFormatter, false);

// TEST(CSP, Move3){




  // for (auto element : on0) {
  //   cout << element.first << endl;
  // }

  // cout << check_true << endl;
  // cout << check_false << endl;
  // cout << check_action << endl;
  // cout << check_state << endl;


  //         loc1                              loc2
  // move(onDs_0[i], clDs_0[i], clLoc2s_0[j], onDs_1[i], clLoc1s_0[j], clLoc2s[k])

  // move(disk, loc1, loc2) -> move disk[1, 2, 3] from loc1[1,2,3,A,B,C] to loc2[1,2,3,A,B,C]

  // for disk in disks:
  //   for loc1 in locs:
  //     for loc2 in locs:
  //       move(disk, loc1, loc2) 

  // we know that all binary factors will simiply be 0, 1 so when constructed these
  // we wouldn't have to change
  // what about factors that have multiple possible values?
  // should we construct new factors for each one of them?
  // yes, but how?
  
  // Move move12b(dkeys);
  // DecisionTreeFactor actual = move12b.toDecisionTreeFactor();


  // actual.dot("actual");
  // DecisionTreeFactor f1(
  //     Cl1_0 & ClB_0 & On1_0 & Cl1_1 & ClB_1 & On1_1,
  //     "0 0 0  0 0 1  0 1 0   0 0 1  0 0 0  1 0 0   0 1 0  1 0 0  0 0 0");
  // EXPECT(assert_equal(f1, actual));



    // Move_act(9, nrActions);
  // DiscreteKey Move_act(9, nrActions);

  // Check that a single value is equal to a decision stump with only one "1":
  // SingleValue singleValue1(On1_0, 1); // 0, 1 ,2, 3, 4, 5 => 1, 2, 3, a, b, c
  // SingleValue singleValue2(On2_0, 2);
  // SingleValue singleValue3(On3_0, 3);
  // SingleValue singleValue4(Cl1_0, 0); // 0, 1 => true, false
  // SingleValue singleValue5(Cl2_0, 1);
  // SingleValue singleValue6(Cl3_0, 1);
  // SingleValue singleValue7(ClA_0, 1);
  // SingleValue singleValue8(ClB_0, 0);
  // SingleValue singleValue9(ClC_0, 0);
  // SingleValue singleValue10(Move_act, 10); // 0 => 1(what), 1(from), 1(to) 

  // Instantiate discrete values

  // DiscreteKey On1_0(0, nrDisks + nrTowers), On2_0(1, nrDisks + nrTowers), On3_0(2, nrDisks + nrTowers),
  //   Cl1_0(3, nrBool), Cl2_0(4, nrBool), Cl3_0(5, nrBool), ClA_0(6, nrBool), ClB_0(7, nrBool),
  //   ClC_0(8, nrBool);
  // vector<DiscreteKey> dkeys_t0{On1_0, On2_0, On3_0, Cl1_0, Cl2_0, Cl3_0, ClA_0, ClB_0, ClC_0};

  // Move move(dkeys_t0);

  // move.print();
  // move.toDecisionTreeFactor();

  // DiscreteKey Move(9, nrActions), On1_1(10, nrDisks + nrTowers), On2_1(11, nrDisks + nrTowers), 
  //   On3_1(12, nrDisks + nrTowers), Cl1_1(13, nrBool), Cl2_1(14, nrBool), Cl3_1(15, nrBool),
  //   ClA_1(16, nrBool), ClB_1(17, nrBool), ClC_1(18, nrBool);

  // vector<DiscreteKey> dkeys_t1{On1_1, On2_1, On3_1, Cl1_1, Cl2_1, Cl3_1, ClA_1, ClB_1, ClC_1};

  // Check that a single value is equal to a decision stump with only one "1":
  // SingleValue singleValue(AZ, 2);
  // DecisionTreeFactor f1(AZ, "0 0 1");
  // EXPECT(assert_equal(f1, singleValue.toDecisionTreeFactor()));

  // // Create domains
  // Domains domains;
  // domains.emplace(0, Domain(ID));
  // domains.emplace(1, Domain(AZ));
  // domains.emplace(2, Domain(UT));

  // // Ensure arc-consistency: just wipes out values in AZ domain:
  // EXPECT(singleValue.ensureArcConsistency(1, &domains));
  // LONGS_EQUAL(3, domains.at(0).nrValues());
  // LONGS_EQUAL(1, domains.at(1).nrValues());
  // LONGS_EQUAL(3, domains.at(2).nrValues());

// }
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}