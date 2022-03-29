/*
 * MutexConstraint.cpp
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/SingleValueConstraint.h>
#include <cpp/planning/OperatorConstraint.h>
#include <cpp/planning/NullOperatorConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
OperatorConstraint::OperatorConstraint(const DiscreteKeys& dkeys,
  const vector<MultiValueConstraint>& factors) 
: DiscreteFactor(dkeys.indices()) {
  for (const MultiValueConstraint& factor: factors) factors_.push_back(factor);
  dkeys_ = dkeys;
  dkey_ = dkeys.back();
  cardinality_ = dkey_.second;
}

/* ************************************************************************* */
void OperatorConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorConstraint on "
      << formatter(dkey_.first) << endl;
}
/* ************************************************************************* */
double OperatorConstraint::operator()(const DiscreteValues& values) const {
  NullOperatorConstraint null(dkeys_);
  return factors_[values.at(dkey_.first)](values) || null(values);
}

/* ************************************************************************* */
double add_(const double& a, const double& b) {
    return a + b;
}

DecisionTreeFactor OperatorConstraint::toDecisionTreeFactor() const {
  vector<DecisionTreeFactor> multiplied_fs;
  // null operator
  // SingleValueConstraint s_const(dkey_, 0);
  // NullOperatorConstraint null(dkeys_);
  // DecisionTreeFactor null_tree = null.toDecisionTreeFactor();
  // DecisionTreeFactor multiplied_null = s_const.toDecisionTreeFactor() * null_tree;
  
  // vector<double> null_table;
  // for (size_t i=0; i < cardinality_; i++) {
  //   if (i == 0) null_table.push_back(1.0);
  //   else null_table.push_back(0.0);
  // }
  // DecisionTreeFactor null(dkey_, null_table);
  // multiplied_fs.push_back(null);

  // SingleValueConstraint s_const(dkey_, 0);
  // DecisionTreeFactor multiplied = s_const.toDecisionTreeFactor();
  // multiplied_fs.push_back(multiplied);

  // operators
  for (size_t i=0; i < cardinality_; i++) {
    SingleValueConstraint s_const(dkey_, i);
    DecisionTreeFactor multiplied = s_const.toDecisionTreeFactor() * factors_[i].toDecisionTreeFactor();
    multiplied_fs.push_back(multiplied);
  }
  
  // combine into one huge tree
  DecisionTreeFactor added_f;
  vector<double> table;
  for (size_t i=0; i < multiplied_fs.size(); i++) {
    added_f = added_f.create();
  }

  return added_f;  
}

/* ************************************************************************* */
DecisionTreeFactor OperatorConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
