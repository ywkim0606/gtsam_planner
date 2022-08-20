/*
 * BinarySameConstraint.h
 * @brief domain constraint
 * @date Mar 20, 2022
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

namespace gtsam_planner {

/**
 * Binary same constraint: ensures two variables have same values.
 */
class BinarySameConstraint : public DiscreteFactor {
  size_t cardinality0_;
  size_t cardinality1_;

 public:

  BinarySameConstraint();

  BinarySameConstraint(const DiscreteKey& key1, const DiscreteKey& key2);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const BinarySameConstraint*>(&other))
      return false;
    else {
      const BinarySameConstraint& f(static_cast<const BinarySameConstraint&>(other));
      return (cardinality0_ == f.cardinality0_) &&
             (cardinality1_ == f.cardinality1_);
    }
  }

  /// Calculate value
  double operator()(const DiscreteValues& values) const override;

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Convert into a TableFactor
  TableFactor toTableFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /// Multiply into a decisiontree
  TableFactor operator*(const TableFactor& f) const override;

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
