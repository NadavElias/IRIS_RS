#include "drone_planner.h"

extern bool state_sample; // Nadav

namespace drone {

DronePlanner::DronePlanner(const RobotPtr& robot, const EnvPtr& env, const Idx seed)
     : robot_(robot), env_(env), seed_(seed)
{
    rng_.seed(seed_);
    uni_ = RealUniformDist(0, 1);
    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
}

void DronePlanner::SampleStartConfig(const Idx max_iter, const Idx seed) {
    Rand rng;
    rng.seed(seed);
    RealUniformDist uni(0, 1);
    
    Vec3 pos;
    RealNum yaw, camera_angle;

    for (auto i = 0; i < max_iter; ++i) {
        for (auto j = 0; j < 3; ++j) {
            auto lo = env_->EnvironmentBoundary(j) - 5;
            auto hi = env_->EnvironmentBoundary(j) + 5;
            pos[j] = uni(rng)*(hi - lo) + lo;
        }

        yaw = uni(rng)*(kMaxYaw - kMinYaw) + kMinYaw;
        camera_angle = uni(rng)*(kMaxCameraAngle - kMinCameraAngle) + kMinCameraAngle;

        robot_->SetConfig(pos, yaw, camera_angle);
        robot_->ComputeShape();

        if (env_->IsCollisionFree(robot_->Config()->Position(), robot_->SphereRadius())
            && env_->IfCorrectDirection(robot_->CameraPos(), robot_->CameraTangent(), robot_->FOV(), validation_distance_)) {     
            robot_->SaveStartConfig();
            std::cout << "Found at " << i << std::endl;
            std::cout << "Start config: ";
            robot_->Config()->Print(std::cout);
            return;
        }
    }

    std::cout << "Fail to find valid start config." << std::endl;
    exit(1);
}

void DronePlanner::SetParams(const RealNum step_size, const bool if_k_nearest) {
    step_size_ = step_size;
    k_nearest_ = if_k_nearest;
}

void DronePlanner::BuildAndSaveInspectionGraph(const String file_name, const Idx target_size) {
    std::cout << "Prepare to build an inspection graph of size: " << target_size << std::endl;

    // State space.
    ompl::RNG::setSeed(seed_);
    auto state_space_ = ob::StateSpacePtr(new DroneStateSpace());

    ob::RealVectorBounds bounds(5);
    for (auto i = 0; i < 3; ++i) {
        bounds.setLow(i, env_->EnvironmentBoundary(i) - 5);
        bounds.setHigh(i, env_->EnvironmentBoundary(i, false) + 5);
    }
    bounds.setLow(3, kMinYaw);
    bounds.setHigh(3, kMaxYaw);
    bounds.setLow(4, kMinCameraAngle);
    bounds.setHigh(4, kMaxCameraAngle);    
    state_space_->as<DroneStateSpace>()->setBounds(bounds);

    // Space info.
    space_info_.reset(new ob::SpaceInformation(state_space_));
    space_info_->setStateValidityCheckingResolution(0.01);
    using namespace std::placeholders;
    space_info_->setStateValidityChecker(std::bind(&DronePlanner::StateValid, this, _1));
    space_info_->setup();

    // Problem definition.
    auto problem_def = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info_));
    ob::State *start = space_info_->allocState();
    state_space_->as<DroneStateSpace>()->StateFromConfiguration(start, robot_->StartConfig());

    problem_def->addStartState(start);
    auto goal = ob::GoalPtr(new InfiniteGoal(space_info_));
    problem_def->setGoal(goal);
    auto obj = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(space_info_));
    problem_def->setOptimizationObjective(obj);

    // Planner.
    auto planner = ob::PlannerPtr(new og::RRG(space_info_));
    planner->as<og::RRG>()->setRange(step_size_);
    planner->as<og::RRG>()->setGoalBias(0.0);
    planner->as<og::RRG>()->setKNearest(k_nearest_);
    planner->setProblemDefinition(problem_def);
    planner->setup();

    // Build graph incrementally.
    auto num_targets = env_->NumTargets();
    Inspection::Graph *graph = new Inspection::Graph();

    #if USE_POI_FOCUS
    graph_ = graph; // Nadav
    valid_states_counter = 0;
    invalid_states_counter = 0;
    focus_frequency = FOCUS_FREQUENCY; //FOCUS_FREQUENCY
    focus_min_extend = FOCUS_MIN_EXTEND;
    #endif // USE_POI_FOCUS
    
    ob::PlannerData tree_data(space_info_);
    ob::PlannerData graph_data(space_info_);

    while (graph->NumVertices() < target_size) {
        BuildRRGIncrementally(graph, planner, tree_data, graph_data);
        std::cout << "Covered targets: " << graph->NumTargetsCovered() 
            << ", " << graph->NumTargetsCovered()*(RealNum)100/num_targets << "%" << std::endl;
    }

    graph->Save(file_name, true);

    delete graph;

    #if USE_POI_FOCUS
    graph_ = nullptr; // Nadav
    #endif USE_POI_FOCUS
}

void DronePlanner::BuildRRGIncrementally(Inspection::Graph *graph, 
                            ob::PlannerPtr& planner, 
                            ob::PlannerData& tree_data, 
                            ob::PlannerData& graph_data) 
{
    // use ompl planner
    const TimePoint start = Clock::now();
    ob::PlannerStatus solved;
    ob::IterationTerminationCondition ptc(incremental_step_);
    solved = planner->solve(ptc);
    planner->as<og::RRG>()->getPlannerData(tree_data);
    planner->as<og::RRG>()->getUncheckedEdges(graph_data);
    SizeType total_time_build = RelativeTime(start);

    SizeType avg_time_build = 0;
    Idx prev_size = graph->NumVertices();
    Idx current_size = tree_data.numVertices();
    if (current_size > prev_size) {
        avg_time_build = SizeType(total_time_build/(current_size - prev_size));
    }
    else {
        return;
    }

    // update inspection graph
    for (Idx i = prev_size; i < current_size; ++i) {
        std::cout << i << " " << std::flush;
        graph->AddVertex(i);
        auto vertex = graph->Vertex(i);
        vertex->state = space_info_->allocState();
        space_info_->copyState(vertex->state, tree_data.getVertex(i).getState());

        const TimePoint start = Clock::now();
        ComputeVisibilitySet(vertex);
        vertex->time_vis = RelativeTime(start);
        vertex->time_build = avg_time_build;

        graph->UpdateGlobalVisibility(vertex->vis);

        // tree edges
        std::vector<unsigned> edges;
        auto num_parent = tree_data.getIncomingEdges(i, edges);
        if (num_parent == 1) {
            Idx p = edges[0];
            Inspection::EPtr edge(new Inspection::Edge(p, i));
            edge->checked = true;
            edge->valid = true;
            edge->cost = space_info_->distance(tree_data.getVertex(p).getState(), tree_data.getVertex(i).getState());
            graph->AddEdge(edge);
        }
        else if (num_parent != 0){
            std::cerr << "More than one parent! Error!" << std::endl;
            exit(1);
        }

        // graph edges
        edges.clear();
        graph_data.getEdges(i, edges);
        for (auto && e : edges) {
            Inspection::EPtr edge(new Inspection::Edge(i, e));
            const ob::State *source = tree_data.getVertex(i).getState();
            const ob::State *target = tree_data.getVertex(e).getState();
            edge->cost = space_info_->distance(source, target);

            if (validate_all_edges_) {
                const TimePoint start = Clock::now();
                bool valid = this->CheckEdge(source, target);
                // edge->checked = true;
                edge->valid = valid;
                edge->time_forward_kinematics = RelativeTime(start);
            }

            graph->AddEdge(edge);
        }
    }
}

void DronePlanner::ComputeVisibilitySet(Inspection::VPtr vertex) const {
    const auto& s = vertex->state->as<DroneStateSpace::StateType>();
    robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
    robot_->ComputeShape();

    auto visible_points = env_->GetVisiblePointIndices(robot_->CameraPos(), 
                                                       robot_->CameraTangent(), 
                                                       robot_->FOV(),
                                                       robot_->MinDOF(),
                                                       robot_->MaxDOF());

    auto& vis_set = vertex->vis;
    vis_set.Clear();

    for (auto p : visible_points) {
        vis_set.Insert(p);
    }
}

bool DronePlanner::StateValid(const ob::State *state) { // Nadav
    const auto& s = state->as<DroneStateSpace::StateType>();

    if (!env_->IsCollisionFree(s->Position(), robot_->SphereRadius())) {
         return false;
    }

    if (RandomRealNumber(0, 1) > reject_ratio_) {
        return true;
    }
    
    robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
    robot_->ComputeShape();

    #if USE_POI_FOCUS
    if (!state_sample)
        return env_->IfCorrectDirection(robot_->CameraPos(),
                                    robot_->CameraTangent(),
                                    robot_->FOV(),
                                    validation_distance_);

    state_sample = false;
    bool extends_view = true;
    bool focus = false;
    if (graph_ != nullptr)
    {
        RealNum coverage = graph_->NumTargetsCovered()*(RealNum)100/env_->NumTargets();
        if (coverage > FOCUS_START_COVERAGE && valid_states_counter % focus_frequency == 0 && invalid_states_counter < FOCUS_MAX_FAILURES)
        {
            focus = true;
            auto visible_points = env_->GetVisiblePointIndices(robot_->CameraPos(), 
                                                            robot_->CameraTangent(), 
                                                            robot_->FOV(),
                                                            robot_->MinDOF(),
                                                            robot_->MaxDOF());
            VisibilitySet vis_set;
            for (auto p : visible_points) {
                vis_set.Insert(p);
            }

            RealNum extend_rate = vis_set.ContainsMoreThan(graph_->GetGlobalVisibility());
            std::cout << focus_min_extend << " - " << extend_rate*(RealNum)100/env_->NumTargets() << ", ";
            extends_view = extend_rate*(RealNum)100/env_->NumTargets() >= focus_min_extend;
        }
        if (invalid_states_counter == FOCUS_MAX_FAILURES)
        {
            focus_frequency *= 2;
            focus_min_extend /= 2;
            //std::cout << "\nfocus_frequency " << focus_frequency << std::endl;
        }
    }
    bool validity = extends_view && env_->IfCorrectDirection(robot_->CameraPos(),
                                                            robot_->CameraTangent(),
                                                            robot_->FOV(),
                                                            validation_distance_);
    if (validity)
    {
        valid_states_counter++;
        invalid_states_counter = 0;
        if (focus)
        {
            focus_frequency = FOCUS_FREQUENCY;
            //focus_min_extend = FOCUS_MIN_EXTEND;
        }
    }
    else
    {
        invalid_states_counter++;
    }

    return validity;

    #else
    return env_->IfCorrectDirection(robot_->CameraPos(),
                                robot_->CameraTangent(),
                                robot_->FOV(),
                                validation_distance_);
    #endif // USE_POI_FOCUS
}

bool DronePlanner::CheckEdge(const ob::State *source, const ob::State *target) const {
    const auto& s0 = source->as<DroneStateSpace::StateType>();
    const auto& s1 = target->as<DroneStateSpace::StateType>();
    Vec3 p0 = s0->Position();
    Vec3 p1 = s1->Position();

    Idx num_steps = std::ceil((p0 - p1).norm() / 0.5);

    for (Idx i = 1; i < num_steps; ++i) {
        RealNum p = i / (RealNum)num_steps;
        Vec3 config_mid = p0 * p + p1 * (1 - p);

        if (!env_->IsCollisionFree(config_mid, robot_->SphereRadius())) {
            return false;
        }
    }

    return true;
}


RealNum DronePlanner::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound) {
    return uni_(rng_)*(higher_bound - lower_bound) + lower_bound;
}

SizeType DronePlanner::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

}
