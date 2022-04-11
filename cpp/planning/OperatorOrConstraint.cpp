/*
 * OperatorConstraint.cpp
 * @brief OperatorConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/SingleValueConstraint.h>
#include <cpp/planning/OperatorOrConstraint.h>
#include <cpp/planning/MultiValueConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
OperatorOrConstraint::OperatorOrConstraint(const DiscreteKey& dkey,
    const DiscreteKeys& dkeys, const vector<MultiValueConstraint>& factors)
    : DiscreteFactor(dkeys.indices()) {
  for (const MultiValueConstraint& factor: factors) factors_.push_back(factor);
  dkey_ = dkey;
  dkeys_ = dkeys;
  cardinality_ = dkey_.second;
}

/* ************************************************************************* */
void OperatorOrConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorOrConstraint on "
       << "j=" << formatter(dkey_.first) << endl;
}
/* ************************************************************************* */
double OperatorOrConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i=0; i < cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    if (single(values)==1.0 && factors_[i](values)==1.0) return 1.0;
  }
  return 0.0;
}

/* ************************************************************************* */
size_t OperatorOrConstraint::operatorKey() const {
  return dkey_.first;
}

/* ************************************************************************* */
double add_o(const double& a, const double& b) {
    return a + b;
}

DecisionTreeFactor OperatorOrConstraint::toDecisionTreeFactor() const {
  SingleValueConstraint single_comb(dkey_, 0);
  DecisionTreeFactor combined = single_comb * factors_[0].toDecisionTreeFactor();
  for (size_t i=1; i<cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    DecisionTreeFactor single_op = single * factors_[i].toDecisionTreeFactor();
    combined = combined.apply(single_op, &add_o);
  }
  return combined;
}

/* ************************************************************************* */
DecisionTreeFactor OperatorOrConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
