#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_KRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_KRRTSTAR_

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/key_extractors.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <limits>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>

#include <boost/tuple/tuple.hpp>

namespace ompl
{

  namespace geometric
  {

    class KRRTstar
      : public base::Planner
    {
    public:
      KRRTstar(const base::SpaceInformationPtr &si);

      virtual ~KRRTstar();

      virtual void getPlannerData(base::PlannerData &data) const;

      virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

      virtual void clear();

      /** \brief Set the goal bias

          In the process of randomly selecting states in
          the state space to attempt to go towards, the
          algorithm may in fact choose the actual goal state, if
          it knows it, with some probability. This probability
          is a real number between 0.0 and 1.0; its value should
          usually be around 0.05 and should not be too large. It
          is probably a good idea to use the default value. */
      void setGoalBias(double goalBias)
      {
        goalBias_ = goalBias;
      }

      /** \brief Get the goal bias the planner is using */
      double getGoalBias() const
      {
        return goalBias_;
      }

      /** \brief Set a different nearest neighbors datastructure */
      template<template<typename T> class NN>
      void setNearestNeighbors()
      {
        nn_.reset(new NN<Motion*>());
      }

      /** \brief Option that delays collision checking procedures.
          When it is enabled, all neighbors are sorted by cost. The
          planner then goes through this list, starting with the lowest
          cost, checking for collisions in order to find a parent. The planner
          stops iterating through the list when a collision free parent is found.
          This prevents the planner from collsion checking each neighbor, reducing
          computation time in scenarios where collision checking procedures are expensive.*/
      void setDelayCC(bool delayCC)
      {
        delayCC_ = delayCC;
      }

      /** \brief Get the state of the delayed collision checking option */
      bool getDelayCC() const
      {
        return delayCC_;
      }

      virtual void setup();

      ///////////////////////////////////////
      // Planner progress property functions
      std::string getIterationCount() const
      {
        return boost::lexical_cast<std::string>(iterations_);
      }
      std::string getBestCost() const
      {
        return boost::lexical_cast<std::string>(bestCost());
      }

    protected:

      /** \brief Representation of a motion */
      class Motion
      {
      public:
        Motion(const base::SpaceInformationPtr &si)
          : si_(si),
            state(si->allocState()),
            parent(NULL)
        { }

        ~Motion()
        {
          si_->freeState(state);
        }

        const base::SpaceInformationPtr si_;

        /** \brief The state contained by the motion */
        base::State       *state;

        /** \brief The parent motion in the exploration tree */
        Motion            *parent;
        base::Cost        incCost;

        base::Cost        g;
        base::Cost        lmc;

        std::set<Motion*> children;
      };

      /** \brief Free the memory allocated by this planner */
      void freeMemory();

      /** \brief Compute distance between motions (actually distance between contained states) */
      double distanceFunction(const Motion *a, const Motion *b) const
      {
        return si_->distance(a->state, b->state);
      }

      bool isFinite(const base::Cost& c) const
      {
        return opt_->isCostBetterThan(c, opt_->infiniteCost());
      }

      std::pair<base::Cost, base::Cost> get_key(const Motion* a) const
      {
        const auto& lmc = a->lmc;
        const auto& h = base::goalRegionCostToGo(a->state, pdef_->getGoal().get());
        return std::make_pair(opt_->combineCosts(lmc, h), lmc);
      }

      void removeFromParent(Motion *m);

      void deleteMotion(Motion *m);

      /** \brief Deletes (frees memory) the motion and its children motions. */
      void deleteBranch(Motion *motion);

      base::Cost bestCost() const
      {
        if(!best_motion_)
          return opt_->infiniteCost();

        return best_motion_->lmc;
      }

      /** \brief Computes the Cost To Go heuristically as the cost to come from start to motion plus
          the cost to go from motion to goal. If \e shortest is true, the estimated cost to come
          start-motion is given. Otherwise, this cost to come is the current motion cost. */
      base::Cost costToGo(const Motion *motion, const bool shortest = true) const;

      /** \brief State sampler */
      base::StateSamplerPtr                          sampler_;

      /** \brief A nearest-neighbors datastructure containing the tree of motions */
      boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

      /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
      double                                         goalBias_;

      /** \brief The random number generator */
      RNG                                            rng_;

      /** \brief Option to delay and reduce collision checking within iterations */
      bool                                           delayCC_;

      /** \brief Objective we're optimizing */
      base::OptimizationObjectivePtr                 opt_;

      /** \brief A list of states in the tree that satisfy the goal condition */
      std::set<Motion*>                              goalMotions_;

      /** \brief Stores the Motion containing the last added initial start state. */
      Motion *                                       startMotion_;

      // 전체 버텍스의 집합
      std::set<Motion*> vertices_;

      //////////////////////////////
      // Planner progress properties
      /** \brief Number of iterations the algorithm performed */
      unsigned int                                   iterations_;
      /** \brief Best cost found so far by algorithm */

      Motion* best_motion_;
    };
  }
}

#endif
