/*
 * MutexConstraint.cpp
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/OperatorConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_example {

/* ************************************************************************* */
OperatorConstraint(const DiscreteKeys& dkeys,
                const vector<DecisionTreeFactor> factors)
  : DiscreteFactor(dkeys.indices()) {
    for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
    for (const DecisionTreeFactor& factor: factors) factors_.push_back(factor)
  }

/* ************************************************************************* */
void OperatorConstraint::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << s << "OperatorConstraint on ";
  for (Key dkey : keys_) std::cout << formatter(dkey) << " ";
  std::cout << std::endl;
}
/* ************************************************************************* */
double OperatorConstraint::operator()(const DiscreteValues& values) const {
  for (const DecisionTreeFactor& factor: factors_) {
    if (factor(values) == 1.0) return 1.0;
  }
  return 0.0;
}
/* ************************************************************************* */
// DecisionTreeFactor OperatorConstraint::toDecisionTreeFactor() const {
//   size_t nrKeys = keys_.size();
//   keys += DiscreteKey(keys_[0], cardinality_);
//   vector<double> table;
//   for (size_t i1 = 0; i1 < cardinality_; i1++) table.push_back(i1 == value_);
//   DecisionTreeFactor converted(keys, table);
//   return converted;
// }

/* ************************************************************************* */
// DecisionTreeFactor OperatorConstraint::operator*(const DecisionTreeFactor& f) const {
//   // TODO: can we do this more efficiently?
//   return toDecisionTreeFactor() * f;
// }

}  // namespace gtsam_example
