/*
 * MultiValueConstraint.cpp
 * @brief domain constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
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
  dkeys_ = dkeys;
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
  for (const size_t& value : values) values_.push_back(value);
}

/* ************************************************************************* */
void MultiValueConstraint::print(const string& s,
  const KeyFormatter& formatter) const {
  cout << s << "MultiValueConstraint on ";
  for (Key dkey : keys_) cout << formatter(dkey) << " ";
  cout << ::endl;
}

/* ************************************************************************* */
DiscreteKeys MultiValueConstraint::discreteKeys() const {
  return dkeys_;
}

/* ************************************************************************* */
double MultiValueConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i=0; i < values_.size(); i++) {
    if (values.count(keys_[i]) != 0) {
      size_t value = values.at(keys_[i]);
      if (value != values_[i]) return 0.0;
    }
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
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
}  // namespace gtsam
