/*
 * MutexConstraint.cpp
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/OrConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_example {

/* ************************************************************************* */
OrConstraint::OrConstraint(const vector<DecisionTreeFactor>& factors) 
: DiscreteFactor() {
  for (const DecisionTreeFactor& factor: factors) factors_.push_back(factor);
  set<DiscreteKey> dkeys_set;
  for (DecisionTreeFactor factor : factors) {
    DiscreteKeys dkeys = factor.discreteKeys();
    for (DiscreteKey dkey : dkeys) {
      dkeys_set.insert(dkey);
    }
  }
  dkeys_ = {dkeys_set.begin(), dkeys_set.end()};
  for (const DiscreteKey& dkey : dkeys_) cardinalities_.insert(dkey);
}

/* ************************************************************************* */
void OrConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OrConstraint on ";
  for (DiscreteKey dkey : dkeys_) std::cout << formatter(dkey.first) << " ";
  cout << endl;
}
/* ************************************************************************* */
double OrConstraint::operator()(const DiscreteValues& values) const {
  for (const DecisionTreeFactor factor: factors_) {
    if (factor(values) == 1.0) return 1.0;
  }
  return 0.0;
}

/* ************************************************************************* */
// double validate() {
  
// }
DecisionTreeFactor OrConstraint::toDecisionTreeFactor() const {
  vector<double> table;
  const auto assignments = DiscreteValues::CartesianProduct(dkeys_);
  for (const DiscreteValues assignment : assignments) {
    size_t is_true = 0;
    for (DecisionTreeFactor factor : factors_) {
      is_true += factor(assignment);
    }
    table.push_back(is_true > 0);
  }
  DiscreteKeys rev_dkeys = {dkeys_.rbegin(), dkeys_.rend()};
  DecisionTreeFactor converted(rev_dkeys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor OrConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_example
