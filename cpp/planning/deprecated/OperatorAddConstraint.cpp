/*
 * OperatorConstraint.cpp
 * @brief OperatorConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/SingleValueConstraint.h>
#include <cpp/planning/OperatorAddConstraint.h>
#include <cpp/planning/MultiValueConstraint.h>
#include <cpp/planning/NullConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
OperatorAddConstraint::OperatorAddConstraint(const DiscreteKey& dkey,
    const DiscreteKeys& dkeys, const vector<MultiValueConstraint>& factors,
    const vector<NullConstraint>& null_factors) : DiscreteFactor(dkeys.indices()) {
  for (const MultiValueConstraint& factor: factors) factors_.push_back(factor);
  for (const NullConstraint& factor: null_factors) null_factors_.push_back(factor);
  dkey_ = dkey;
  dkeys_ = dkeys;
  cardinality_ = dkey_.second;
}

/* ************************************************************************* */
void OperatorAddConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorAddConstraint on "
       << "j=" << formatter(dkey_.first) << endl;
}
/* ************************************************************************* */
double OperatorAddConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i=0; i < cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    if (single(values)==1.0 && factors_[i](values)==1.0
      && null_factors_[i](values)==1.0) return 1.0;
  }
  return 0.0;
}

/* ************************************************************************* */
size_t OperatorAddConstraint::operatorKey() const {
  return dkey_.first;
}

/* ************************************************************************* */
double add_(const double& a, const double& b) {
    return a + b;
}

DecisionTreeFactor OperatorAddConstraint::toDecisionTreeFactor() const {
  vector<DecisionTreeFactor> operators;
  for (size_t i=1; i<cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    DecisionTreeFactor single_op = single * factors_[i].toDecisionTreeFactor()
      * null_factors_[i].toDecisionTreeFactor();
    operators.push_back(single_op);
  }

  SingleValueConstraint single_comb(dkey_, 0);
  DecisionTreeFactor combined = single_comb * factors_[0].toDecisionTreeFactor()
    * null_factors_[0].toDecisionTreeFactor();
  for (DecisionTreeFactor single_op : operators) {
    combined = combined.apply(single_op, &add_);
  }

  return combined;
}

/* ************************************************************************* */
DecisionTreeFactor OperatorAddConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
