/*
 * OperatorOrConstraint.h
 * @brief OperatorOrConstraint constraint
 * @date Mar 20, 2022
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
 * Operator constraint: true if out of one or more OperatorConstraint is true.
 */
class OperatorOrConstraint : public DiscreteFactor {
  vector<MultiValueConstraint> factors_;  /// < all possible operators
  DiscreteKeys dkeys_;
  DiscreteKey dkey_;
  size_t cardinality_;

  public:

  /// Construct from factors.
  OperatorOrConstraint(const DiscreteKey& dkey, const DiscreteKeys& dkeys,
    const vector<MultiValueConstraint>& factors);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const OperatorOrConstraint*>(&other))
      return false;
    else {
      const OperatorOrConstraint& f(dynamic_cast<const OperatorOrConstraint&>(other));
      if (cardinality_ == f.cardinality_) {
        for (size_t i = 0; i < factors_.size(); i++) {
          if (factors_[i].equals(f.factors_[i], 1e-9) == false) return false;
        }
        return true;
      }
      return false;
    }
  }

  /// Return the key associated with which operator to use.
  size_t operatorKey() const;

  /// Calculate value
  double operator()(const DiscreteValues& values) const override;

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override;

  /// Convert into a TableFactor
  TableFactor toTableFactor() const override;

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override;

  /// Multiply into a TableFactor
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
