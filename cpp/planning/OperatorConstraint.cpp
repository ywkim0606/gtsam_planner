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

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_example {

/* ************************************************************************* */
OperatorConstraint::OperatorConstraint(const DiscreteKey& dkey,
  const vector<DecisionTreeFactor>& factors) 
: DiscreteFactor(boost::assign::cref_list_of<1>(dkey.first)) {
  for (const DecisionTreeFactor& factor: factors) factors_.push_back(factor);
  dkey_ = dkey;
  cardinality_ = dkey.second;
}

/* ************************************************************************* */
void OperatorConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OperatorConstraint on "
      << formatter(dkey_.first) << endl;
}
/* ************************************************************************* */
double OperatorConstraint::operator()(const DiscreteValues& values) const {
  for (const DecisionTreeFactor factor: factors_) {
    if (factor(values) == 1.0) return 1.0;
  }
  return 0.0;
}

double add_(const double& a, const double& b) {
    return a + b;
}

/* ************************************************************************* */
DecisionTreeFactor OperatorConstraint::toDecisionTreeFactor() const {
  // TODO: Add with singlevalue constraint
  vector<DecisionTreeFactor> multiplied_fs;
  for (size_t i=0; i < cardinality_; i++) {
    SingleValueConstraint s_const = SingleValueConstraint(dkey_, i);
    DecisionTreeFactor multiplied = s_const.toDecisionTreeFactor() * factors_[i];
    multiplied_fs.push_back(multiplied);
  }

  DecisionTreeFactor added_f = multiplied_fs[0];
  for (size_t i=1; i < multiplied_fs.size(); i++) {
    added_f = added_f.apply(multiplied_fs[i], &add_);
  }
  return added_f;

  // vector<double> table;
  // const auto assignments = DiscreteValues::CartesianProduct(dkeys_);
  // for (const DiscreteValues assignment : assignments) {
  //   size_t is_true = 0;
  //   for (DecisionTreeFactor factor : factors_) {
  //     if (factor(assignment) == 1.0) 
  //     is_true += factor(assignment);
  //   }
  //   table.push_back(is_true > 0);
  // }
  // DiscreteKeys rev_dkeys = {dkeys_.rbegin(), dkeys_.rend()};
  // DecisionTreeFactor converted(rev_dkeys, table);
  // return converted;
  
}

/* ************************************************************************* */
DecisionTreeFactor OperatorConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_example
