/*
 * OperatorConstraint.cpp
 * @brief OperatorConstraint constraint
 * @date Apr 3, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/OperatorConstraint.h>
#include <cpp/planning/MultiValueConstraint.h>
#include <cpp/planning/NullConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
OperatorConstraint::OperatorConstraint(const DiscreteKeys& multi_keys,
  const vector<size_t>& values, const DiscreteKeys& null_keys, const DiscreteKeys& dkeys)
  : DiscreteFactor(dkeys.indices()) {
  multi_ = multi_keys;
  values_ = values;
  null_ = null_keys;
  dkeys_ = dkeys;
  // multi_constraint_ = MultiValueConstraint(multi_keys, values);
  // null_constraint_ = NullOperatorConstraint(null_keys);

}

/* ************************************************************************* */
void OperatorConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorConstraint on ";
  for (DiscreteKey dkey : dkeys_) cout << formatter(dkey.first) << " ";
  cout << endl;
}

/* ************************************************************************* */
DiscreteKeys OperatorConstraint::discreteKeys() const {
  return dkeys_;
}

/* ************************************************************************* */
double OperatorConstraint::operator()(const DiscreteValues& values) const {
  MultiValueConstraint multi_constraint(multi_, values_);
  NullConstraint null_constraint(null_);
  return multi_constraint(values) && null_constraint(values);
}

/* ************************************************************************* */
DecisionTreeFactor OperatorConstraint::toDecisionTreeFactor() const {
  MultiValueConstraint multi_constraint(multi_, values_);
  NullConstraint null_constraint(null_);
  return multi_constraint.toDecisionTreeFactor() * null_constraint.toDecisionTreeFactor();
}

/* ************************************************************************* */
DecisionTreeFactor OperatorConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
