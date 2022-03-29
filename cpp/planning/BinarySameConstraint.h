/*
 * SingleValue.h
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/DiscreteKey.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>

using namespace gtsam;
namespace gtsam_planner {

/**
 * SingleValue constraint: ensures a variable takes on a certain value.
 * This could of course also be implemented by changing its `Domain`.
 */
class BinarySameConstraint : public DiscreteFactor {
  size_t cardinality0_, cardinality1_;

 public:

  /// Construct from key, cardinality, and given value.
  BinarySameConstraint(const DiscreteKey& key1, const DiscreteKey& key2)
      : DiscreteFactor(boost::assign::cref_list_of<2>(key1.first)(key2.first)),
        cardinality0_(key1.second),
        cardinality1_(key2.second) {}

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