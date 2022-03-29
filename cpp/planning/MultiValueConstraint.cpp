/*
 * SingleValue.cpp
 * @brief domain constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/MultiValueConstraint.h>
#include <cpp/planning/SingleValueConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
MultiValueConstraint::MultiValueConstraint(const DiscreteKeys& dkeys,
  const vector<size_t>& values) : DiscreteFactor(dkeys.indices()) {
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
  for (const size_t& value : values) values_.push_back(value);
}

/* ************************************************************************* */
void MultiValueConstraint::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << s << "MultiValueConstraint on ";
  for (Key dkey : keys_) std::cout << formatter(dkey) << " ";
  std::cout << std::endl;
}

/* ************************************************************************* */
double MultiValueConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i=0; i < values_.size(); i++) {
    size_t value = values.at(keys_[i]);  // get the value for that key
    if (value != values_[i]) return 0.0;
  }
  return 1.0;
}

/* ************************************************************************* */
DecisionTreeFactor MultiValueConstraint::toDecisionTreeFactor() const {
  DecisionTreeFactor converted;
  size_t nrKeys = keys_.size();
  for (size_t i = 0; i < nrKeys; i++) {
    SingleValueConstraint single(discreteKey(i), values_[i]);
    converted = converted * single.toDecisionTreeFactor();
  }
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor MultiValueConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
}  // namespace gtsam
