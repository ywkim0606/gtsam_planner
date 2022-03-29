/*
 * MutexConstraint.h
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <cpp/planning/MultiValueConstraint.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace gtsam;
namespace gtsam_planner {

/**
 * Operator constraint: choose an operator = factors_[values].
 */
class NullOperatorConstraint : public DiscreteFactor {
  map<Key, size_t> cardinalities_;
  DiscreteKeys dkeys_;
  DiscreteKeys dkeys_t_;
  DiscreteKeys dkeys_tp_;

 public:

  /// Construct from factors.
  NullOperatorConstraint(const DiscreteKeys& dkeys);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const NullOperatorConstraint*>(&other))
      return false;
    else {
      const NullOperatorConstraint& f(static_cast<const NullOperatorConstraint&>(other));
      return cardinalities_.size() == f.cardinalities_.size() &&
              std::equal(cardinalities_.begin(), cardinalities_.end(),
                          f.cardinalities_.begin());
    }
  }

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
