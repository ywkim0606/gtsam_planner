/*
 * MutexAddConstraint.cpp
 * @brief Mutually exclusive constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/MutexAddConstraint.h>
#include <cpp/planning/NotSingleValueConstraint.h>
#include <cpp/planning/SingleValueConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
MutexAddConstraint::MutexAddConstraint(const DiscreteKey& dkey, 
  const DiscreteKeys& dkeys, const vector<size_t>& values) 
  : DiscreteFactor(dkeys.indices()) {
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
  for (const size_t& value : values) values_.push_back(value);
  dkey_ = dkey;
  dkeys_ = std::vector<DiscreteKey>(dkeys.begin() + 1, dkeys.end());;
}

/* ************************************************************************* */
void MutexAddConstraint::print(const std::string& s, const KeyFormatter& formatter) const {
  cout << s << "MutexAddConstraint on "
       << "j=" << formatter(dkey_.first) << endl;
}

/* ************************************************************************* */
double MutexAddConstraint::operator()(const DiscreteValues& values) const {
  size_t count = 0;
  for (size_t i=0; i < values_.size(); i++) {
    size_t value = values.at(dkeys_[i].first);
    if (value == values_[i]) count++;
    if (count > 1.0) return 0.0;
  }
  return 1.0;
}

/* ************************************************************************* */
double add_m(const double& a, const double& b) {
    return a + b;
}

/* ************************************************************************* */
DecisionTreeFactor MutexAddConstraint::toDecisionTreeFactor() const {
  // vector of decision tree factors
  vector<DecisionTreeFactor> factors;
  // number of keys
  size_t nrKeys = dkeys_.size();
  for (size_t i = 0; i < nrKeys; i++) {
    SingleValueConstraint single(dkeys_[i], values_[i]);
    DecisionTreeFactor singleTree = single.toDecisionTreeFactor();
    for (size_t j = 0; j < nrKeys; j++) {
      if (i == j) continue;
      NotSingleValueConstraint notSingle(dkeys_[j], values_[j]);
      singleTree = singleTree * notSingle.toDecisionTreeFactor();
    }
    factors.push_back(singleTree);
  }

  SingleValueConstraint single_comb(dkey_, 0);
  DecisionTreeFactor combined = single_comb * factors[0];
  size_t nrFactors = factors.size();
  for (size_t i=1; i<nrFactors; i++) {
    SingleValueConstraint single(dkey_, i);
    DecisionTreeFactor single_m = single * factors[i];
    combined = combined.apply(single_m, &add_m);
  }
  return combined;
}

/* ************************************************************************* */
DecisionTreeFactor MutexAddConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
}  // namespace gtsam
