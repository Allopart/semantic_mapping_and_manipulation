#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_QRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_QRRTSTAR_

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

    class QRRTstar
      : public base::Planner
    {
    public:
      QRRTstar(const base::SpaceInformationPtr &si);

      virtual ~QRRTstar();

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

      void setAlpha(const double& alpha)
      {
        alpha_ = alpha;
      }

      double setAlpha() const
      {
        return alpha_;
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

      bool checkMotion(const Motion* a, const Motion* b);

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

      void replan();
      void update_queue(Motion *m);

      void remove_unpromising();

      void reduce_unreachable();

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

      double alpha_;

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

      struct nbh_edge_t {
        base::Cost cost;
        bool need_cd;
      };

      using nbh_map = std::unordered_map<Motion*, nbh_edge_t>;
      using nbh_t = std::unordered_map<Motion*, nbh_map>;

      nbh_t out_neighbor_;
      nbh_t in_neighbor_;

      // collision cache
      using collide_map_t = std::unordered_map<const Motion*,
                                               std::unordered_set<const Motion*> >;
      collide_map_t collide_from_;
      collide_map_t collide_to_;

      // queue: Motion*으로도 찾을 수 있어야 하고, key (lmc + h, lmc) 로도 정렬되어있어야함
      struct queue_value
      {
        base::OptimizationObjectivePtr opt_;
        Motion* m;
        std::pair<base::Cost, base::Cost> key;

        bool operator<(const queue_value& rhs) const {
          const auto& k1 = key;
          const auto& k2 = rhs.key;
          return opt_->isCostBetterThan(k1.first, k2.first) ||
                      (!opt_->isCostBetterThan(k2.first, k1.first) && opt_->isCostBetterThan(k1.second, k2.second));
        }
      };

      boost::multi_index::multi_index_container<
        queue_value,
        boost::multi_index::indexed_by<
          boost::multi_index::hashed_unique<boost::multi_index::member<queue_value, Motion*, &queue_value::m> >,
          boost::multi_index::ordered_non_unique<boost::multi_index::identity<queue_value> >
          > > queue_;


      // unreachables_: Motion*으로도 찾을 수 있어야하고 삽입된 순서도 알 수 있어야함
      boost::multi_index::multi_index_container<
        Motion*,
        boost::multi_index::indexed_by<
          boost::multi_index::sequenced<>,
          boost::multi_index::hashed_unique<boost::multi_index::identity<Motion*> >
          > > unreachables_;

    };
  }
}

#endif
