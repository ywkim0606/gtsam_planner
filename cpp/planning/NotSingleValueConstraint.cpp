/*
 * NotSingleValueConstraint.cpp
 * @brief NotSingleValueConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/NotSingleValueConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
void NotSingleValueConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "NotSingleValueConstraint on "
       << "j=" << formatter(keys_[0]) << " with value " << value_ << endl;
}

/* ************************************************************************* */
double NotSingleValueConstraint::operator()(const DiscreteValues& values) const {
  if (values.at(keys_[0]) != value_) return 1.0;
  return 0.0;
}

/* ************************************************************************* */
DecisionTreeFactor NotSingleValueConstraint::toDecisionTreeFactor() const {
  DiscreteKeys keys;
  keys += DiscreteKey(keys_[0], cardinality_);
  vector<double> table;
  for (size_t i1 = 0; i1 < cardinality_; i1++) {
    if (i1 == value_) table.push_back(0.0);
    else table.push_back(1.0);
  }
  DecisionTreeFactor converted(keys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor NotSingleValueConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
}  // namespace gtsam
