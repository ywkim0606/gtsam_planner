/*
 * BinarySameConstraint.cpp
 * @brief domain constraint
 * @date Mar 20, 2022
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/BinarySameConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
void BinarySameConstraint::print(const string& s, const KeyFormatter& formatter) const {
    std::cout << s << "BinarySameConstraint on " << formatter(keys_[0]) << " and "
              << formatter(keys_[1]) << std::endl;
}

/* ************************************************************************* */
double BinarySameConstraint::operator()(const DiscreteValues& values) const {
  return (double)(values.at(keys_[0]) == values.at(keys_[1]));
}

/* ************************************************************************* */
DecisionTreeFactor BinarySameConstraint::toDecisionTreeFactor() const {
  DiscreteKeys keys;
  keys.push_back(DiscreteKey(keys_[0], cardinality0_));
  keys.push_back(DiscreteKey(keys_[1], cardinality1_));
  std::vector<double> table;
  for (size_t i1 = 0; i1 < cardinality0_; i1++)
    for (size_t i2 = 0; i2 < cardinality1_; i2++) table.push_back(i1 == i2);
  DecisionTreeFactor converted(keys, table);
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor BinarySameConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}


/* ************************************************************************* */
}  // namespace gtsam
