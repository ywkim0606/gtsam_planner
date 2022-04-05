/*
 * OperatorChooseConstraint.cpp
 * @brief OperatorChooseConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/OperatorChooseConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
OperatorChooseConstraint::OperatorChooseConstraint(const vector<MultiValueConstraint>& factors,
  size_t which_op) : DiscreteFactor(factors[which_op].discreteKeys().indices()) {
  for (const MultiValueConstraint& factor: factors) factors_.push_back(factor);
  dkeys_ = factors_[which_op].discreteKeys();
  which_op_ = which_op;
}

/* ************************************************************************* */
void OperatorChooseConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorChooseConstraint on ";
  for (DiscreteKey dkey : dkeys_) cout << formatter(dkey.first) << " ";
  cout << endl;
}

/* ************************************************************************* */
double OperatorChooseConstraint::operator()(const DiscreteValues& values) const {
  return factors_[which_op_](values);
}

/* ************************************************************************* */
DecisionTreeFactor OperatorChooseConstraint::toDecisionTreeFactor() const {
  DecisionTreeFactor converted = factors_[which_op_].toDecisionTreeFactor();
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor OperatorChooseConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
