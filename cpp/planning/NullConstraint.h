/*
 * NullConstraint.h
 * @brief NullConstraint constraint
 * @date Mar 20, 2022
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
 * NullConstraint constraint: This operator checks if the states did not change.
 * Checks variables at state_t and state_t+1 have same values.
 */
class NullConstraint : public DiscreteFactor {
  map<Key, size_t> cardinalities_;
  DiscreteKeys dkeys_;
  DiscreteKeys dkeys_t_;
  DiscreteKeys dkeys_tp_;
  DiscreteKey discreteKey(size_t i) const {
  Key j = keys_[i];
  return DiscreteKey(j, cardinalities_.at(j));
  }

 public:
 
  // /// Default constructor for I/O
  // NullConstraint();
  
  /// Construct from factors.
  NullConstraint(const DiscreteKeys& dkeys);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const NullConstraint*>(&other))
      return false;
    else {
      const NullConstraint& f(static_cast<const NullConstraint&>(other));
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
