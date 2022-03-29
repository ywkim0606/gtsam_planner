/*
 * MutexConstraint.cpp
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Yoonwoo Kim
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <cpp/planning/NullOperatorConstraint.h>
#include <cpp/planning/BinarySameConstraint.h>

#include <boost/make_shared.hpp>
using namespace gtsam;
using namespace std;

namespace gtsam_planner {

/* ************************************************************************* */
NullOperatorConstraint::NullOperatorConstraint(const DiscreteKeys& dkeys)
  : DiscreteFactor(dkeys.indices()) {
  dkeys_ = dkeys;
  for (const DiscreteKey& dkey : dkeys) cardinalities_.insert(dkey);
  for (size_t i = 0; i < dkeys.size(); i++) {
    if (i < dkeys.size()/2) dkeys_t_.push_back(dkeys[i]);
    else dkeys_tp_.push_back(dkeys[i]);
  }
}

/* ************************************************************************* */
void NullOperatorConstraint::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << "NullOperatorConstraint on ";
  for (Key dkey : keys_) cout << formatter(dkey) << " ";
  cout << endl;
}

/* ************************************************************************* */
double NullOperatorConstraint::operator()(const DiscreteValues& values) const {
  for (size_t i = 0; i < dkeys_t_.size(); i++) {
    if (values.at(dkeys_t_[i].first) != values.at(dkeys_tp_[i].first)) return 0.0;
  }
  return 1.0;
}

/* ************************************************************************* */
DecisionTreeFactor NullOperatorConstraint::toDecisionTreeFactor() const {
  // const vector<DiscreteValues> assignments = DiscreteValues::CartesianProduct(dkeys_);
  // vector<double> table;
  // for(auto assignment: assignments) {
  //   size_t half = assignment.size() / 2;
  //   size_t is_same = 1;
  //   for (size_t i=0; i<assignment.size()-half; i++) {
  //     is_same *= (assignment.at(i) == assignment.at(i+half));      
  //   }
  //   table.push_back(is_same);
  // }
  // DecisionTreeFactor converted(dkeys_, table);
  // return converted;
  // const std::vector<DiscreteValues> assignments = DiscreteValues::CartesianProduct(dkeys_);
  // vector<double> table;
  // for(auto assignment: assignments) {
  //   size_t half = assignment.size() / 2;
  //   size_t is_same = 1;
  //   for (auto it=assignment.begin(); it != assignment.end(); it++) {
  //     size_t key_t = it->first;
  //     size_t key_tp = it->first + half;
  //     if (key_tp > assignment.end()->first+1) break;
  //     is_same *= int(assignment.at(key_t) == assignment.at(key_tp));
  //   }
  //   table.push_back(is_same);
  // }
  // DecisionTreeFactor converted(dkeys_, table);
  // return converted;
  DecisionTreeFactor converted;
  size_t nrKeys = dkeys_.size();
  for (size_t i=0; i<dkeys_t_.size(); i++) {
    BinarySameConstraint bsame(dkeys_t_[i], dkeys_tp_[i]);
    converted = converted * bsame.toDecisionTreeFactor();
  }
  return converted;
}

/* ************************************************************************* */
DecisionTreeFactor NullOperatorConstraint::operator*(const DecisionTreeFactor& f) const {
  // TODO: can we do this more efficiently?
  return toDecisionTreeFactor() * f;
}

}  // namespace gtsam_planner
