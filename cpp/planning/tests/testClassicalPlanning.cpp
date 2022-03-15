/*
 * testClassicalPlanning.cpp
 * @brief develop code for Classical Planning solver
 * @date Nov 3, 2021
 * @author Yoonwoo Kim
 */

#include <cpp/planning/OrConstraint.h>

#include <boost/assign/std/map.hpp>

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace boost::assign;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(OrConstraint, GenerateOrConstraint) {
  DiscreteKey A(0,2), B(1, 2);
  DiscreteKey C(1, 2), D(2, 2);
  DecisionTreeFactor f_and(A & B, "0 0 0 1");
  DecisionTreeFactor f_or(C & D, "0 1 1 1");

  // for (auto it = f_and.begin(); it != f_and.end(); ++it)
  //   cout << ' ' << *it;
  vector<DecisionTreeFactor> factors;
  factors.push_back(f_and);
  factors.push_back(f_or);

  set<DiscreteKey> dkeys_set;
  for (DecisionTreeFactor& factor : factors) {
    DiscreteKeys dkeys = factor.discreteKeys();
    for (DiscreteKey& dkey : dkeys) {
      dkeys_set.insert(dkey);
    }
  }
  DiscreteKeys dkeys;
  dkeys = {dkeys_set.begin(), dkeys_set.end()};

  // for(auto elem: dkeys) {
  //   cout << elem.first << ", ";
  // }

  // const auto assignments = DiscreteValues::CartesianProduct(dkeys);
  // for(auto assignment: assignments) {
  //   cout << f_and(assignment) << ", ";
  // }

  vector<double> table;
  const auto assignments = DiscreteValues::CartesianProduct(dkeys);
  vector<pair<DiscreteValues, double>> result;
  for (const auto& assignment : assignments) {
    double is_true = 0.0;
    for (DecisionTreeFactor& factor : factors) {
      if (factor(assignment) == 1.0) is_true++;
    }
    if (is_true >= 1.0) table.push_back(1.0);
    else table.push_back(0.0);
  }
  DecisionTreeFactor converted(dkeys, table);

  converted.print();
  // converted.dot("orFactor", DefaultKeyFormatter, false);
  DecisionTreeFactor multiplied = f_and * f_or;
  // multiplied.dot("andFactor", DefaultKeyFormatter, false);

  DiscreteValues values;
  values[0] = 1; //
  values[1] = 0; //
  values[2] = 1; //

  cout << converted(values) << endl;
  cout << multiplied(values) << endl;
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}