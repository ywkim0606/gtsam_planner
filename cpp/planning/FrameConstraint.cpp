/*
 * FrameConstraint.cpp
 * @brief FrameConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <cpp/planning/SingleValueConstraint.h>
#include <cpp/planning/FrameConstraint.h>
#include <cpp/planning/NullConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
FrameConstraint::FrameConstraint(const DiscreteKey& dkey,
    const DiscreteKeys& dkeys, const vector<NullConstraint>& factors)
    : DiscreteFactor(dkeys.indices()) {
  for (const NullConstraint& factor: factors) factors_.push_back(factor);
  dkey_ = dkey;
  dkeys_ = dkeys;
  cardinality_ = dkey_.second;
}

/* ************************************************************************* */
void FrameConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "FrameConstraint on "
       << "j=" << formatter(dkey_.first) << endl;
}
/* ************************************************************************* */
double FrameConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i=0; i < cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    if (single(values)==1.0 && factors_[i](values)==1.0) return 1.0;
  }
  return 0.0;
}

/* ************************************************************************* */
size_t FrameConstraint::operatorKey() const {
  return dkey_.first;
}

/* ************************************************************************* */
double add_f(const double& a, const double& b) {
    return a + b;
}

DecisionTreeFactor FrameConstraint::toDecisionTreeFactor() const {
  vector<DecisionTreeFactor> operators;
  SingleValueConstraint single_comb(dkey_, 0);
  DecisionTreeFactor combined = single_comb * factors_[0].toDecisionTreeFactor();
  for (size_t i=1; i<cardinality_; i++) {
    SingleValueConstraint single(dkey_, i);
    DecisionTreeFactor single_op = single * factors_[i].toDecisionTreeFactor();
    combined = combined.apply(single_op, &add_f);
  }
  return combined;
}

/* ************************************************************************* */
TableFactor FrameConstraint::toTableFactor() const {
  DecisionTreeFactor dt = toDecisionTreeFactor();
  return dt.toTableFactor();
}

/* ************************************************************************* */
DecisionTreeFactor FrameConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
TableFactor FrameConstraint::operator*(const TableFactor& f) const {
  // TODO: can we do this more efficiently?
  return toTableFactor() * f;
}

}  // namespace gtsam_planner
