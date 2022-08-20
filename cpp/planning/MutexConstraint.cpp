/*
 * MutexConstraint.cpp
 * @brief Mutually exclusive constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <cpp/planning/MutexConstraint.h>
#include <cpp/planning/NotSingleValueConstraint.h>
#include <cpp/planning/SingleValueConstraint.h>
#include <cpp/planning/OrConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
MutexConstraint::MutexConstraint(const DiscreteKeys& dkeys,
  const vector<size_t>& values) : DiscreteFactor(dkeys.indices()) {
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
  for (const size_t& value : values) values_.push_back(value);
}

/* ************************************************************************* */
void MutexConstraint::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << s << "MutexConstraint on ";
  for (Key dkey : keys_) std::cout << formatter(dkey) << " ";
  std::cout << std::endl;
}

/* ************************************************************************* */
double MutexConstraint::operator()(const DiscreteValues& values) const {
  size_t count = 0;
  for (size_t i=0; i < values_.size(); i++) {
    size_t value = values.at(keys_[i]);
    if (value == values_[i]) count++;
    if (count > 1.0) return 0.0;
  }
  return 1.0;
}

/* ************************************************************************* */
DecisionTreeFactor MutexConstraint::toDecisionTreeFactor() const {
  vector<DecisionTreeFactor> factors;
  size_t nrKeys = keys_.size();
  for (size_t i = 0; i < nrKeys; i++) {
    SingleValueConstraint single(discreteKey(i), values_[i]);
    DecisionTreeFactor singleTree = single.toDecisionTreeFactor();
    for (size_t j = 0; j < nrKeys; j++) {
      if (i == j) continue;
      NotSingleValueConstraint notSingle(discreteKey(j), values_[j]);
      singleTree = singleTree * notSingle.toDecisionTreeFactor();
    }
    factors.push_back(singleTree);
  }
  OrConstraint mutex(factors);
  DecisionTreeFactor mutex_tree = mutex.toDecisionTreeFactor();
  return mutex_tree;
}

/* ************************************************************************* */
TableFactor MutexConstraint::toTableFactor() const {
  vector<TableFactor> factors;
  size_t nrKeys = keys_.size();
  for (size_t i = 0; i < nrKeys; i++) {
    SingleValueConstraint single(discreteKey(i), values_[i]);
    TableFactor singleTable = single.toTableFactor();
    for (size_t j = 0; j < nrKeys; j++) {
      if (i == j) continue;
      NotSingleValueConstraint notSingle(discreteKey(j), values_[j]);
      singleTable = singleTable * notSingle.toTableFactor();
    }
    factors.push_back(singleTable);
  }
  OrConstraint mutex(factors);
  TableFactor mutex_table = mutex.toTableFactor();
  return mutex_table;
}

/* ************************************************************************* */
DecisionTreeFactor MutexConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
TableFactor MutexConstraint::operator*(const TableFactor& f) const {
  return toTableFactor() * f;
}

/* ************************************************************************* */
}  // namespace gtsam
