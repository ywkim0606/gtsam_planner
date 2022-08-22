/*
 * OrConstraint.h
 * @brief OR constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <cpp/planning/OrConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;
namespace gtsam_planner {

/* ************************************************************************* */
OrConstraint::OrConstraint(const vector<DecisionTreeFactor>& factors) 
: DiscreteFactor() {
  // for (const DecisionTreeFactor& factor: factors) factors_.push_back(factor);
  set<DiscreteKey> dkeys_set;
  for (DecisionTreeFactor factor : factors) {
    dt_factors_.push_back(factor);
    table_factors_.push_back(factor.toTableFactor());
    DiscreteKeys dkeys = factor.discreteKeys();
    for (DiscreteKey dkey : dkeys) dkeys_set.insert(dkey);
  }
  dkeys_ = {dkeys_set.begin(), dkeys_set.end()};
  for (const DiscreteKey& dkey : dkeys_) cardinalities_.insert(dkey);
}

/* ************************************************************************* */
OrConstraint::OrConstraint(const vector<TableFactor>& factors) 
: DiscreteFactor() {
  // for (const TableFactor& factor: factors) factors_.push_back(factor);
  set<DiscreteKey> dkeys_set;
  for (TableFactor factor : factors) {
    table_factors_.push_back(factor);
    dt_factors_.push_back(factor.toDecisionTreeFactor());
    DiscreteKeys dkeys = factor.discreteKeys();
    for (DiscreteKey dkey : dkeys) dkeys_set.insert(dkey);
  }
  dkeys_ = {dkeys_set.begin(), dkeys_set.end()};
  for (const DiscreteKey& dkey : dkeys_) cardinalities_.insert(dkey);
}

/* ************************************************************************* */
void OrConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "OrConstraint on ";
  for (DiscreteKey dkey : dkeys_) std::cout << formatter(dkey.first) << " ";
  cout << endl;
}
/* ************************************************************************* */
double OrConstraint::operator()(const DiscreteValues& values) const {
  if (!dt_factors_.empty()) {
    for (DecisionTreeFactor factor: dt_factors_) {
      if (factor(values) == 1.0) return 1.0;
    }
  } else {
    for (TableFactor factor: table_factors_) {
      if (factor(values) == 1.0) return 1.0;
    }
  }
  return 0.0;
}

/* ************************************************************************* */
DecisionTreeFactor OrConstraint::toDecisionTreeFactor() const {
  if (dt_factors_.empty()) return DecisionTreeFactor();
  vector<double> table;
  const auto assignments = DiscreteValues::CartesianProduct(dkeys_);
  for (const DiscreteValues assignment : assignments) {
    size_t is_true = 0;
    for (DecisionTreeFactor factor : dt_factors_) {
      if (factor(assignment) == 1.0) 
      is_true += factor(assignment);
    }
    table.push_back(is_true > 0);
  }
  DiscreteKeys rev_dkeys = {dkeys_.rbegin(), dkeys_.rend()};
  return DecisionTreeFactor(rev_dkeys, table);
}

/* ************************************************************************* */
TableFactor OrConstraint::toTableFactor() const {
  if (table_factors_.empty()) return TableFactor();
  vector<double> table;
  const auto assignments = DiscreteValues::CartesianProduct(dkeys_);
  for (const DiscreteValues assignment : assignments) {
    size_t is_true = 0;
    for (TableFactor factor : table_factors_) {
      if (factor(assignment) == 1.0) 
      is_true += factor(assignment);
    }
    table.push_back(is_true > 0);
  }
  DiscreteKeys rev_dkeys = {dkeys_.rbegin(), dkeys_.rend()};
  return TableFactor(rev_dkeys, table);
}

/* ************************************************************************* */
DecisionTreeFactor OrConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

/* ************************************************************************* */
TableFactor OrConstraint::operator*(const TableFactor& f) const {
  return toTableFactor() * f;
}

}  // namespace gtsam_planner
