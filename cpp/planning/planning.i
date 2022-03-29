
namespace gtsam_planner {

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

#include <cpp/planning/NotSingleValueConstraint.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
virtual class NotSingleValueConstraint : gtsam::DiscreteFactor {
  NotSingleValueConstraint(gtsam::Key key, size_t n, size_t value);
  NotSingleValueConstraint(const gtsam::DiscreteKey& dkey, size_t value);
  
  void print(const string s="",
            const gtsam::KeyFormatter& formatter = 
              gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/MultiValueConstraint.h>
virtual class MultiValueConstraint : gtsam::DiscreteFactor {
  MultiValueConstraint(const gtsam::DiscreteKeys& dkey, const std::vector<size_t>& values);
  
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/OrConstraint.h>
virtual class OrConstraint : gtsam::DiscreteFactor {
  OrConstraint(const std::vector<gtsam::DecisionTreeFactor>& factors);
  
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/MutexConstraint.h>
virtual class MutexConstraint : gtsam::DiscreteFactor {
  MutexConstraint(const gtsam::DiscreteKeys& dkey, const std::vector<size_t>& values);
  
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/OperatorConstraint.h>
virtual class OperatorConstraint : gtsam::DiscreteFactor {
  OperatorConstraint(const gtsam::DiscreteKeys& dkeys, const std::vector<gtsam_planner::MultiValueConstraint>& factors);
  
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/NullOperatorConstraint.h>
virtual class NullOperatorConstraint : gtsam::DiscreteFactor {
  NullOperatorConstraint(const gtsam::DiscreteKeys& dkeys);
  
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

#include <cpp/planning/BinarySameConstraint.h>
virtual class BinarySameConstraint : gtsam::DiscreteFactor {
  BinarySameConstraint(const gtsam::DiscreteKey& key1, const gtsam::DiscreteKey& key2);
  void print(const string s="",
          const gtsam::KeyFormatter& formatter = 
            gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::DiscreteFactor& other, double tol) const;
  double operator()(const gtsam::DiscreteValues& values) const;
  gtsam::DecisionTreeFactor toDecisionTreeFactor() const;
};

}
