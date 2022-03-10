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

#include <boost/assign.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace gtsam;
namespace gtsam_example {

/**
 * Operator constraint: choose an operator = factors_[values].
 */
class OperatorConstraint : public DiscreteFactor {
  std::map<Key, size_t> cardinalities_t_;
  DiscreteKey discreteKey(size_t i) const
  size_t cardinality_;  /// < Number of values
  vector<DecisionTreeFactor> factors_;  /// < all possible operators

  DiscreteKey discreteKey() const {
    return DiscreteKey(keys_[0], cardinality_);
  }

 public:

  /// Construct from DiscreteKey and given value.
  OperatorConstraint(const DiscreteKey& dkey, const vector<DecisionTreeFactor> factors);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const OperatorConstraint*>(&other))
      return false;
    else {
      const OperatorConstraint& f(static_cast<const OperatorConstraint&>(other));
      return (cardinality_ == f.cardinality_) &&
              std::equal(factors_.begin(), factors_.end(), f.factors_.begin());
              // std::equal(values_.begin(), values_.end(), f.values_.begin()) &&
              // std::equal(factors_.begin(), factors_.end(), f.factors_.begin());
    }
  }

  /// Calculate value
  double operator()(const DiscreteValues& values) const override;

  // /// Convert into a decisiontree
  // DecisionTreeFactor toDecisionTreeFactor() const override;

  // /// Multiply into a decisiontree
  // DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;  

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

}  // namespace gtsam_example