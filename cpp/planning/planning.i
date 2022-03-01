// gtsam Discrete
class gtsam::DiscreteKey;
class gtsam::DiscreteFactor;
virtual class gtsam::DecisionTreeFactor;
virtual class gtsam::DiscreteValues;

namespace gtsam_example {

#include <cpp/planning/SingleValueConstraint.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
virtual class SingleValueConstraint : gtsam::DiscreteFactor {
  SingleValueConstraint(gtsam::Key key, size_t n, size_t value);
  SingleValueConstraint(const gtsam::DiscreteKey& dkey, size_t value);
  
  void print(const string s="",
            const gtsam::KeyFormatter& formatter = 
              gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

}
