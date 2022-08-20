/*
 * ValidVariablesConstraint.cpp
 * @brief ValidVariablesConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/MultiValueConstraint.h>
#include <cpp/planning/BinarySameConstraint.h>
#include <cpp/planning/ValidVariablesConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
ValidVariablesConstraint::ValidVariablesConstraint(const BinarySameConstraint& b_factor,
    const vector<MultiValueConstraint>& m_factors, const DiscreteKeys& dkeys) 
: DiscreteFactor(dkeys.indices()) {
  b_factor_ = b_factor;
  for (const MultiValueConstraint& factor: m_factors) m_factors_.push_back(factor);
  dkeys_ = dkeys;

}

/* ************************************************************************* */
void ValidVariablesConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "ValidVariablesConstraint on ";
  for (DiscreteKey dkey : dkeys_) std::cout << formatter(dkey.first) << " ";
  cout << endl;
}
/* ************************************************************************* */
double ValidVariablesConstraint::operator()(const DiscreteValues& values) const {
  for (const MultiValueConstraint factor: m_factors_) {
    if (factor(values) == 1.0) return 1.0;
  }
  if (b_factor_(values) == 1.0) return 1.0;
  return 0.0;
}

/* ************************************************************************* */
DecisionTreeFactor ValidVariablesConstraint::toDecisionTreeFactor() const {
  vector<double> table;
  const auto assignments = DiscreteValues::CartesianProduct(dkeys_);
  for (const DiscreteValues assignment : assignments) {
    size_t is_true = 0;
    for (MultiValueConstraint factor : m_factors_) {
      if (factor(assignment) == 1.0) is_true += factor(assignment);
    }
    if (b_factor_(assignment) == 1.0) is_true += b_factor_(assignment);
    table.push_back(is_true > 0);
  }
  DiscreteKeys rev_dkeys = {dkeys_.rbegin(), dkeys_.rend()};
  DecisionTreeFactor converted(rev_dkeys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor ValidVariablesConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
