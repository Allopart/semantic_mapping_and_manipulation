#include "qrrt_star/KRRTstar.h"

#include <boost/array.hpp>
#include <boost/chrono.hpp>
#include <boost/math/constants/constants.hpp>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <random>

#include <set>
#include <limits>

#include <cassert>


ompl::geometric::KRRTstar::KRRTstar(const base::SpaceInformationPtr &si) :
  base::Planner(si, "KRRTstar"),
  goalBias_(0.05),
  delayCC_(true),
  iterations_(0),
  best_motion_(nullptr)
{
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;
  specs_.canReportIntermediateSolutions = true;

  Planner::declareParam<double>("goal_bias", this, &KRRTstar::setGoalBias, &KRRTstar::getGoalBias, "0.:.05:1.");

  addPlannerProgressProperty("iterations INTEGER",
                             boost::bind(&KRRTstar::getIterationCount, this));
  addPlannerProgressProperty("best cost REAL",
                             boost::bind(&KRRTstar::getBestCost, this));
}

ompl::geometric::KRRTstar::~KRRTstar()
{
  freeMemory();
}

void ompl::geometric::KRRTstar::setup()
{
  Planner::setup();

  if(pdef_) {
    if(pdef_->hasOptimizationObjective()) {
      opt_ = pdef_->getOptimizationObjective();
    }
    else {
      OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.",
                  getName().c_str());
      opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
  }
  else {
    OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
    setup_ = false;
  }
}

void ompl::geometric::KRRTstar::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();

  goalMotions_.clear();

  const auto vertices = vertices_;
  for(auto& m : vertices) {
    deleteMotion(m);
  }
  vertices_.clear();

  iterations_ = 0;
}

ompl::base::PlannerStatus ompl::geometric::KRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  base::Goal                  *goal   = pdef_->getGoal().get();
  base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

  // 시작 지점 설정
  while (const base::State *st = pis_.nextStart()) {
    Motion *motion = new Motion(si_);

    si_->copyState(motion->state, st);
    motion->parent = NULL;
    motion->incCost = motion->g = motion->lmc = opt_->identityCost();

    startMotion_ = motion; // startMotion_은 모션 딱 하나가 아니라 모션의 집합이어야하지 않나? 얘네 왜케 븅신같이 짰냐
    vertices_.insert(startMotion_);

    double distanceFromGoal;
    if(goal->isSatisfied(startMotion_->state, &distanceFromGoal)) {
      goalMotions_.emplace(startMotion_);
    }

    break;
  }

  if(vertices_.empty()) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  // e+e/d.  K-nearest RRT*
  const double k_rrg = boost::math::constants::e<double>() +
    (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

  OMPL_INFORM("%s: goalBias : %lf", getName().c_str(), goalBias_);
  OMPL_INFORM("%s: goal_s->canSample() : %d", getName().c_str(), goal_s->canSample());

  typedef boost::chrono::steady_clock clock;
  boost::optional<clock::time_point> time_first_solution_found;
  const clock::time_point start_time = clock::now();

  // goal에서 샘플 먼저 찍고 시작하자 (최대 0.1 초 동안, 10개까지만)
  std::vector<base::State*> goal_samples;

  while(ptc == false && ((clock::now() - start_time) < boost::chrono::milliseconds(100))) {
    if(goal_samples.size() >= 10) {
      break;
    }

    const base::State *st = pis_.nextGoal(ptc);
    if(!st) {
      OMPL_INFORM("%s:  can't sample in goal point", getName().c_str());
      break; // Can't sample in goal point
    }

    if(!si_->isValid(st)) {
      continue;
    }

    base::State* p_st = si_->allocState();
    si_->copyState(p_st, st);

    goal_samples.emplace_back(p_st);
  }

  if(goal_samples.empty()) {
    OMPL_INFORM("%s: Found no sample in goal area", getName().c_str());
  }
  else {
    OMPL_INFORM("%s: %lf sec to find a sample in goal area", getName().c_str(),
                boost::chrono::duration_cast<boost::chrono::milliseconds>(clock::now() - start_time).count() / 1000.);
  }

  Motion *rmotion        = new Motion(si_);
  base::State *rstate    = rmotion->state;

  std::size_t          statesGenerated = 0;

  while (ptc == false) {
    // if(time_first_solution_found) {
    if(time_first_solution_found &&
       (clock::now() - *time_first_solution_found > boost::chrono::milliseconds(1000))) { // 1초 더 찾자
      OMPL_INFORM("%s: Solution is found, stopping. total iteration : %d", getName().c_str(), iterations_);
      break;
    }

    iterations_++;

    // sample random state (with goal biasing)
    // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.

    if(!goal_samples.empty()) {
      si_->copyState(rstate, goal_samples.back());
      si_->freeState(goal_samples.back());
      goal_samples.pop_back();
    }
    else if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else {
      sampler_->sampleUniform(rstate);
    }

    if(opt_->isCostBetterThan(bestCost(), costToGo(rmotion))) {
      continue;
    }

    // valid하지 않은 샘플은 버린다.
    if(!si_->isValid(rstate)) {
      continue;
    }

    // Find nearby neighbors of the new motion
    struct md {
      Motion* nbh_m;
      double distance;

      bool operator<(const md& rhs) const { return distance < rhs.distance; }
    };

    std::vector<md> near_distance;

    for(const auto& m : vertices_) {
      near_distance.push_back(md{m, si_->distance(m->state, rstate)});
    }
    std::sort(near_distance.begin(), near_distance.end());

    const std::size_t k = std::ceil(k_rrg * log((double)(vertices_.size() + 1)));

    near_distance.erase(std::next(std::begin(near_distance), std::min(k, near_distance.size())),
                        std::end(near_distance));

    if(!si_->checkMotion(near_distance.begin()->nbh_m->state, rstate)) {
      continue;
    }

    // 새 버텍스를 생성
    Motion *m_new = new Motion(si_);
    si_->copyState(m_new->state, rstate);
    m_new->parent = NULL;
    m_new->incCost = m_new->g = m_new->lmc = opt_->infiniteCost();
    vertices_.emplace(m_new);

    { // Add the new motion to the goalMotion_ list, if it satisfies the goal
      double distanceFromGoal;
      if (goal->isSatisfied(m_new->state, &distanceFromGoal)) {
        goalMotions_.emplace(m_new);
      }
    }

    { // 부모를 고르자
      for(auto it = std::begin(near_distance), e = std::end(near_distance); it != e; ++it) {
        const auto& m_from = it->nbh_m;

        const auto& incCost = opt_->motionCost(m_from->state, m_new->state);
        const auto& lmc_near = opt_->combineCosts(m_from->lmc, incCost);

        if(opt_->isCostBetterThan(lmc_near, m_new->lmc)) {
          if(si_->checkMotion(m_from->state, m_new->state)) {
            removeFromParent(m_new);
            m_new->parent = m_from;
            m_new->parent->children.emplace(m_new);
            m_new->lmc = lmc_near;
            m_new->incCost = incCost;
          }
        }
      }
    }

    // 부모가 없으면 버텍스 삭제하고 다시 샘플
    if(!m_new->parent) {
      deleteMotion(m_new);
      continue;
    }

    { // 리와이어링
      for(auto it = std::begin(near_distance), e = std::end(near_distance); it != e; ++it) {
        const auto& m_to = it->nbh_m;

        const auto& incCost = opt_->motionCost(m_new->state, m_to->state);
        const auto& lmc_near = opt_->combineCosts(m_new->lmc, incCost);

        if(opt_->isCostBetterThan(lmc_near, m_to->lmc)) {
          if(si_->checkMotion(m_new->state, m_to->state)) {
            removeFromParent(m_to);
            m_to->parent = m_new;
            m_to->parent->children.emplace(m_to);
            m_to->lmc = lmc_near;
          }
        }
      }
    }

    // m_new가 unpromising이면 삭제
    if(opt_->isCostBetterThan(base::Cost(1.1 * bestCost().value()), get_key(m_new).first)) {
      deleteBranch(m_new);
    }

    // 솔루션이 있는지를 찾자.
    if(!goalMotions_.empty()) {
      const auto it = std::min_element(std::begin(goalMotions_), std::end(goalMotions_),
                                       [&](const Motion* x, const Motion* y) {
                                         return opt_->isCostBetterThan(x->lmc, y->lmc);
                                       });

      best_motion_ = *it;

      if(!time_first_solution_found) {
        time_first_solution_found = clock::now();
        OMPL_INFORM("%s: The initial solution is found, %lf sec, cost : %lf",
                    getName().c_str(),
                    boost::chrono::duration_cast<boost::chrono::milliseconds>(*time_first_solution_found - start_time).count() / 1000.,
                    bestCost()
                    );
      }
    }
  } // while (ptc == false)

  OMPL_INFORM("%s: solution cost : %lf", getName().c_str(), bestCost());

  bool addedSolution = false;

  if(isFinite(bestCost())) {
    ptc.terminate();
    // construct the solution path
    std::vector<Motion*> mpath;
    Motion* solution = best_motion_;

    while(solution != NULL)
      {
        mpath.push_back(solution);
        solution = solution->parent;
      }

    // set the solution path
    PathGeometric *geoPath = new PathGeometric(si_);
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
      geoPath->append(mpath[i]->state);

    base::PathPtr path(geoPath);
    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    // Does the solution satisfy the optimization objective?
    psol.setOptimized(opt_, bestCost(), false);
    pdef_->addSolutionPath(psol);

    addedSolution = true;
  }

  delete rmotion;

  OMPL_INFORM("%s: Total %u states. %u goal states in tree.",
              getName().c_str(),
              vertices_.size(), goalMotions_.size());

  return base::PlannerStatus(addedSolution, false);
}

void ompl::geometric::KRRTstar::removeFromParent(Motion *m)
{
  if(m->parent) {
    m->parent->children.erase(m);
    m->parent = nullptr;
  }
}

void ompl::geometric::KRRTstar::freeMemory()
{
  const auto& vertices = vertices_; // since deleteMotion deletes the motion from vertices_, copy it
  for(auto& m : vertices) {
    deleteMotion(m);
  }
}

void ompl::geometric::KRRTstar::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (nn_)
    nn_->list(motions);

  // if (lastGoalMotion_)
  //   data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
      if (motions[i]->parent == NULL)
        data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
      else
        data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                     base::PlannerDataVertex(motions[i]->state));
    }
}

void ompl::geometric::KRRTstar::deleteMotion(Motion *m)
{
  // parent의 자식목록에서 제거
  removeFromParent(m);

  for(auto& m_child : m->children) {
    m_child->parent = nullptr;
  }

  // vertices_에서 제거
  vertices_.erase(m);

  goalMotions_.erase(m);

  delete m;
}

void ompl::geometric::KRRTstar::deleteBranch(Motion *motion)
{
  std::deque<Motion *> toDelete;

  toDelete.push_back(motion);

  while(!toDelete.empty()) {
    auto m = toDelete.front();
    toDelete.pop_front();

    for(const auto& child : m->children) {
      toDelete.push_back(child);
    }

    deleteMotion(m);
  }
}

ompl::base::Cost ompl::geometric::KRRTstar::costToGo(const Motion *motion, const bool shortest) const
{
  base::Cost costToCome;
  if (shortest)
    costToCome = opt_->motionCost(startMotion_->state, motion->state); // h_s
  else
    costToCome = motion->lmc; //d_s

  const base::Cost costToGo = base::goalRegionCostToGo(motion->state, pdef_->getGoal().get()); // h_g
  return opt_->combineCosts(costToCome, costToGo); // h_s + h_g
}
