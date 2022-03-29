/*
 * SingleValue.cpp
 * @brief domain constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/SingleValueConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {



/* ************************************************************************* */
void SingleValueConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "SingleValueConstraint on "
       << "j=" << formatter(keys_[0]) << " with value " << value_ << endl;
}

/* ************************************************************************* */
double SingleValueConstraint::operator()(const DiscreteValues& values) const {
  return (double)(values.at(keys_[0]) == value_);
}

/* ************************************************************************* */
DecisionTreeFactor SingleValueConstraint::toDecisionTreeFactor() const {
  DiscreteKeys keys;
  keys += DiscreteKey(keys_[0], cardinality_);
  vector<double> table;
  for (size_t i1 = 0; i1 < cardinality_; i1++) table.push_back(i1 == value_);
  DecisionTreeFactor converted(keys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor SingleValueConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}


/* ************************************************************************* */
}  // namespace gtsam
