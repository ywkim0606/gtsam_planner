/*
 * ValidVariablesConstraint.h
 * @brief ValidVariablesConstraint constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <cpp/planning/MultiValueConstraint.h>
#include <cpp/planning/BinarySameConstraint.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace gtsam;

namespace gtsam_planner {

/**
 * ValidVariablesConstraint constraint: 
 */
class ValidVariablesConstraint : public DiscreteFactor {
  BinarySameConstraint b_factor_;
  vector<MultiValueConstraint> m_factors_;
  DiscreteKeys dkeys_;

 public:

  /// Construct from factors.
  ValidVariablesConstraint(const BinarySameConstraint& b_factor,
    const vector<MultiValueConstraint>& m_factors, const DiscreteKeys& dkeys);

  // print
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const ValidVariablesConstraint*>(&other))
      return false;
    else {
      const ValidVariablesConstraint& f(static_cast<const ValidVariablesConstraint&>(other));
      for (size_t i=0; i < m_factors_.size(); i++) {
        if (m_factors_[i].equals(f.m_factors_[i], 1e-9) == false) {
          return false;
        }
      }
      return b_factor_.equals(f.b_factor_, 1e-9);
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
