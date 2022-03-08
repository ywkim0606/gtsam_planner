/*
 * MutexConstraint.cpp
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/MutexConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_example {

/* ************************************************************************* */
MutexConstraint(const DiscreteKeys& dkeys, const vector<size_t> values)
  : DiscreteFactor(dkeys.indices()) {
    for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
    for (const size_t val& : values) values_.push_back(val);
  }

/* ************************************************************************* */
void MutexConstraint::print(const std::string& s, const KeyFormatter& formatter) const {
  std::cout << s << "MutexConstraint on ";
  for (Key dkey : keys_) std::cout << formatter(dkey) << " ";
  std::cout << std::endl;
}
/* ************************************************************************* */
double MutexConstraint::operator()(const DiscreteValues& values) const {
  size_t num_equal = 0;
  for (size_t i = 0; i < keys_.size(); i++) {
    size_t value = values.at(keys_.at(i));
    if (value == values_.at(i)) num_equal++;
    if (num_equal > 1) return 0.0;
  }
  return 1.0
}
/* ************************************************************************* */
double helper(vector<size_t> values) {
  size_t num_equal = 0;
  for (size_t i = 0; i < keys_.size(); i++) {
    size_t value = values.at(keys_.at(i));
    if (value == values_.at(i)) num_equal++;
    if (num_equal > 1) return 0.0;
  }
  return 1.0
}

DecisionTreeFactor SingleValueConstraint::toDecisionTreeFactor() const {
  size_t nrKeys = keys_.size();
  keys += DiscreteKey(keys_[0], cardinality_);
  vector<double> table;
  for (size_t i1 = 0; i1 < cardinality_; i1++) table.push_back(i1 == value_);
  DecisionTreeFactor converted(keys, table);
  return converted;
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

}  // namespace gtsam_example
