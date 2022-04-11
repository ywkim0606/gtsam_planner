/*
 * OperatorConstraint.h
 * @brief OperatorConstraint constraint
 * @date Apr 3, 2022
 * @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/DiscreteKey.h>

using namespace std;
using namespace gtsam;
namespace gtsam_planner {

/**
 * Operator constraint: create a factor which returns true of state_t and state_tp
 * has values which satisfies the preconditions and effect. It is important that
 * other variables would stay the same if not affected by the operator.
 * i.e) state_t[0] == state_tp[0] if 0th variable is not affected by the precondtions
 * and effect of this operator.
 * Combination of MultiValueConstraint and NullOperatorConstraint
 */

class OperatorConstraint : public DiscreteFactor {
  DiscreteKeys multi_;
  vector<size_t> values_;
  DiscreteKeys null_;
  DiscreteKeys dkeys_;
  // MultiValueConstraint multi_constraint_;
  // NullOperatorConstraint null_constraint_;

 public:

  /// Construct from factors.
  OperatorConstraint(const DiscreteKeys& multi_keys, const vector<size_t>& values,
    const DiscreteKeys& null_keys, const DiscreteKeys& dkeys);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const OperatorConstraint*>(&other))
      return false;
    else {
      const OperatorConstraint& f(static_cast<const OperatorConstraint&>(other));
      return equal(multi_.begin(), multi_.end(), f.multi_.begin()) &&
        equal(values_.begin(), values_.end(), f.values_.begin()) &&
        equal(null_.begin(), null_.end(), f.null_.begin());
      // return multi_constraint_.equals(f.multi_constraint_, 1e-9) &&
      //   null_constraint_.equals(f.null_constraint_, 1e-9);
    }
  }

  /// Return all the discrete keys associated with this factor.
  DiscreteKeys discreteKeys() const;

  /// Calculate value
  double operator()(const DiscreteValues& values) const override;

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;  

    /// Render as markdown table.
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const override {
    return (boost::format("`Constraint` on %1% variables\n") % (size())).str();
  }

  /// Render as html table.
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override {
    return (boost::format("<p>Constraint on %1% variables</p>") % (size())).str();
  }

};

}  // namespace gtsam_planner
