#include "qrrt_star/QRRTstar.h"

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


ompl::geometric::QRRTstar::QRRTstar(const base::SpaceInformationPtr &si) :
  base::Planner(si, "QRRTstar"),
  goalBias_(0.05),
  delayCC_(true),
  iterations_(0),
  best_motion_(nullptr)
{
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;
  specs_.canReportIntermediateSolutions = true;

  Planner::declareParam<double>("goal_bias", this, &QRRTstar::setGoalBias, &QRRTstar::getGoalBias, "0.:.05:1.");

  addPlannerProgressProperty("iterations INTEGER",
                             boost::bind(&QRRTstar::getIterationCount, this));
  addPlannerProgressProperty("best cost REAL",
                             boost::bind(&QRRTstar::getBestCost, this));
}

ompl::geometric::QRRTstar::~QRRTstar()
{
  freeMemory();
}

void ompl::geometric::QRRTstar::setup()
{
  Planner::setup();
  // if(!nn_) {
  //   nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
  // }
  // nn_->setDistanceFunction(boost::bind(&QRRTstar::distanceFunction, this, _1, _2));

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

void ompl::geometric::QRRTstar::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();

  goalMotions_.clear();

  for(auto& m : vertices_) {
    deleteMotion(m);
  }
  vertices_.clear();

  out_neighbor_.clear();
  in_neighbor_.clear();

  queue_.clear();

  iterations_ = 0;
}

ompl::base::PlannerStatus ompl::geometric::QRRTstar::solve(const base::PlannerTerminationCondition &ptc)
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

    auto& in_nbh_m_new = in_neighbor_[m_new];
    auto& out_nbh_m_new = out_neighbor_[m_new];

    // put near vertices to neighbor with deferred cd
    for(const auto& md : near_distance) {
      const auto& nmotion = md.nbh_m; // near motion

      // nmotion -> m_new
      const auto& nb_edge = nbh_edge_t{opt_->motionCost(nmotion->state, m_new->state), true};
      in_nbh_m_new.emplace(nmotion, nb_edge);
      out_neighbor_[nmotion].emplace(m_new, nb_edge);

      // m_new -> nmotion
      const auto& nb_edge_rev = nbh_edge_t{opt_->motionCost(m_new->state, nmotion->state), true};
      out_nbh_m_new.emplace(nmotion, nb_edge_rev);
      in_neighbor_[nmotion].emplace(m_new, nb_edge_rev);
    }

    // 부모 쫓아올라가면서 추가
    for(const auto& md : near_distance) {
      const auto& nmotion = md.nbh_m; // near motion

      auto m_cur = nmotion;
      for(std::size_t i = 0; i < 1; ++i) {
        if(!m_cur->parent) {
          break;
        }

        m_cur = m_cur->parent;

        // m_cur -> m_new
        const auto& nb_edge = nbh_edge_t{opt_->motionCost(m_cur->state, m_new->state), true};

        in_nbh_m_new.emplace(m_cur, nb_edge);
        out_neighbor_[m_cur].emplace(m_new, nb_edge);
      }
    }

    { // 부모를 고르자
      for(auto it = std::begin(in_nbh_m_new), e = std::end(in_nbh_m_new); it != e;) {
        const auto& m_from = it->first;
        auto&       edge = it->second;

        if(!isFinite(m_from->lmc)) { // unreachable이면 부모일 가능성 없음
          ++it;
          continue;
        }

        const auto& lmc_near = opt_->combineCosts(m_from->lmc, edge.cost);

        if(opt_->isCostBetterThan(lmc_near, m_new->lmc)) {
          if(!checkMotion(m_from, m_new)) {
            // 충돌하면 in네이버에서 제거
            it = in_nbh_m_new.erase(it);
            // out_neighbor에서도 삭제
            out_neighbor_[m_from].erase(m_new);
            continue;
          }

          edge.need_cd = false;
          out_neighbor_[m_from][m_new].need_cd = false;

          // 부모 변경
          removeFromParent(m_new);
          m_new->parent = m_from;
          m_new->parent->children.emplace(m_new);
          m_new->lmc = lmc_near;
        }

        ++it;
      }
    }

    // 부모가 있으면 그 부모의 out neighbor에 near를 추가
    if(m_new->parent) {
      const auto& m_parent = m_new->parent;

      auto& out_nbh_m_p = out_neighbor_[m_parent];

      for(const auto& md : near_distance) {
        const auto& nmotion = md.nbh_m; // near motion

        const auto& nb_edge = nbh_edge_t{opt_->motionCost(m_parent->state, nmotion->state), true};

        out_nbh_m_p.emplace(nmotion, nb_edge);
        in_neighbor_[nmotion].emplace(m_parent, nb_edge);
      }

      queue_.insert(queue_value{opt_, m_parent, get_key(m_parent)});
    }

    // queue에 m_new를 넣고 replan
    queue_.insert(queue_value{opt_, m_new, get_key(m_new)});
    replan();


    // m_new가 unreachable이면 unreachables_에 추가
    if(!m_new->parent) {
      unreachables_.push_back(m_new);
    }

    // queue 정리 (queue에 남아있는 애들 중에 reachable한 애들 삭제. branch-and-bound와 효과 비슷)
    // remove_unpromising();

    // reduce unreachable
    reduce_unreachable();

    // 솔루션이 있는지를 찾자.
    if(!goalMotions_.empty()) {
      const auto it = std::min_element(std::begin(goalMotions_), std::end(goalMotions_),
                                       [&](const Motion* x, const Motion* y) {
                                         return opt_->isCostBetterThan(x->lmc, y->lmc);
                                       });

      if(isFinite((*it)->lmc)) {
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

void ompl::geometric::QRRTstar::replan()
{
  while(!queue_.empty()) {
    auto& key_view = queue_.template get<1>();

    if(opt_->isCostBetterThan(bestCost(), key_view.begin()->key.first)) {
      return;
    }

    // pop the minimum key queuevalue
    const auto x = std::move(*key_view.begin());
    key_view.erase(key_view.begin());

    Motion* m = x.m;
    m->g = m->lmc;
    unreachables_.template get<1>().erase(m);

    auto& out_neighbor = out_neighbor_[m];

    for(auto it = std::begin(out_neighbor), e = std::end(out_neighbor); it != e;) {
      const auto& m_to = it->first;
      auto&       edge = it->second;

      const auto& lmc_new = opt_->combineCosts(m->lmc, edge.cost);

      if(opt_->isCostBetterThan(base::Cost(1.05 * lmc_new.value()), m_to->lmc)) {
        if(edge.need_cd && !checkMotion(m, m_to)) {
          it = out_neighbor.erase(it);
          in_neighbor_[m_to].erase(m);
          continue;
        }

        // 충돌 검사를 할 필요가 없음을 표시
        edge.need_cd = false;
        in_neighbor_[m_to][m].need_cd = false;

        // 부모 변경
        m_to->lmc = lmc_new;
        removeFromParent(m_to);
        m_to->parent = m;
        m->children.insert(m_to);

        update_queue(m_to);
      }

      ++it;
    }
  }
}

void ompl::geometric::QRRTstar::update_queue(Motion *m)
{
  // if it's consistent
  if(!opt_->isCostBetterThan(m->lmc, m->g) &&
     !opt_->isCostBetterThan(m->g, m->lmc)) {
    queue_.template get<0>().erase(m);
  }
  else {
    const auto it = queue_.template get<0>().find(m);

    if(it == std::end(queue_)) { // m is not in queue
      queue_.insert(queue_value{opt_, m, get_key(m)});
    }
    else { // m is in queue. update the key
      queue_.modify(it, [&](queue_value& x) { x.key = this->get_key(x.m); });
    }
  }
}

void ompl::geometric::QRRTstar::remove_unpromising()
{
  while(!queue_.empty()) {
    auto& m_view = queue_.template get<0>();

    auto *m_delete = m_view.begin()->m;
    m_view.erase(m_view.begin());

    if(isFinite(m_delete->lmc)) {
      deleteBranch(m_delete);
    }
  }

  queue_.clear();
}

void ompl::geometric::QRRTstar::reduce_unreachable()
{
  for(;;) {
    const auto& size_unreachable = unreachables_.size();
    const auto& size_reachable = vertices_.size() - size_unreachable;

    const auto& min_unreachable = std::size_t{100};
    const auto& max_unreachable = std::size_t(1 * size_reachable);

    if(size_unreachable <= std::max(max_unreachable, min_unreachable)) {
      break;
    }

    const auto& m_oldest = unreachables_.template get<0>().front();

    deleteMotion(unreachables_.template get<0>().front());
  }
}

void ompl::geometric::QRRTstar::removeFromParent(Motion *m)
{
  if(m->parent) {
    m->parent->children.erase(m);
  }
}

void ompl::geometric::QRRTstar::freeMemory()
{
  for(auto& m : vertices_) {
    delete m;
  }
}

bool ompl::geometric::QRRTstar::checkMotion(const Motion* a, const Motion* b)
{
  auto& collide_from_a = collide_from_[a];
  if(collide_from_a.find(b) != collide_from_a.end()) { // already in collision cache
    return false;
  }

  const bool collision_free = si_->checkMotion(a->state, b->state);
  if(!collision_free) {
    collide_from_a.emplace(b);
    collide_to_[b].emplace(a);
  }

  return collision_free;

  // return si_->checkMotion(a->state, b->state);
}

void ompl::geometric::QRRTstar::getPlannerData(base::PlannerData &data) const
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

void ompl::geometric::QRRTstar::deleteMotion(Motion *m)
{
  // neighbor 정리
  for(const auto& x : out_neighbor_[m]) {
    in_neighbor_[x.first].erase(m);
  }
  for(const auto& x : in_neighbor_[m]) {
    out_neighbor_[x.first].erase(m);
  }
  out_neighbor_.erase(m);
  in_neighbor_.erase(m);

  // collision cache에서 제거
  for(const auto& m_to : collide_from_[m]) {
    collide_to_[m_to].erase(m);
  }
  for(const auto& m_from : collide_to_[m]) {
    collide_from_[m_from].erase(m);
  }
  collide_from_.erase(m);
  collide_to_.erase(m);

  // queue에서 제거
  queue_.template get<0>().erase(m);

  // unreachables_에서 제거
  unreachables_.template get<1>().erase(m);

  // parent의 자식목록에서 제거
  removeFromParent(m);

  // vertices_에서 제거
  vertices_.erase(m);

  goalMotions_.erase(m);

  delete m;
}

void ompl::geometric::QRRTstar::deleteBranch(Motion *motion)
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

ompl::base::Cost ompl::geometric::QRRTstar::costToGo(const Motion *motion, const bool shortest) const
{
  base::Cost costToCome;
  if (shortest)
    costToCome = opt_->motionCost(startMotion_->state, motion->state); // h_s
  else
    costToCome = motion->lmc; //d_s

  const base::Cost costToGo = base::goalRegionCostToGo(motion->state, pdef_->getGoal().get()); // h_g
  return opt_->combineCosts(costToCome, costToGo); // h_s + h_g
}
