#pragma once

// Global
#include <algorithm>
#include <iterator>
#include <map>
#include <memory>
#include <stack>

// Local Problem files
#include "grstapse/grstaps_problem_inputs.hpp"
// Local ITAGS Stuff
#include "grstapse/scheduling/milp/milp_scheduler_base.hpp"
#include "grstapse/task_allocation/itags/ditags_base.hpp"

namespace grstapse
{
    // Forward Declarations

    /**!
     * \brief The Dynamic Incremental Task Allocation Graph Search
     *
     * A heuristic search used for trait-based time extended task allocation problems that allows
     * for dynamic reallocation and repair
     *
     *
     */
    class DitagsTetaq : public DitagsBase
    {
       public:
        using Base = DitagsBase;

        /**!
         * \brief Constructor
         *
         * \param parameters input problems
         */
        explicit DitagsTetaq(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs)
            : Base{.problem_inputs = problem_inputs}
            , m_is_closed_nsq_stale(false)
            , m_is_closed_apr_stale(false)
            , m_is_pruned_nsq_stale(false)
            , m_is_pruned_apr_stale(false)
            , m_parent_search(nullptr)
        {}

        /**!
         * \brief Shallow copy just open set
         *
         * @param parameters input problems
         */
        explicit DitagsTetaq(const DitagsTetaq &to_copy, bool is_deep_copy = true)
            : Base{to_copy, is_deep_copy}
        {
            if(is_deep_copy)
            {
                m_parent_search       = nullptr;
                m_is_closed_nsq_stale = to_copy.m_is_closed_nsq_stale;
                m_is_closed_apr_stale = to_copy.m_is_closed_apr_stale;
                m_is_pruned_nsq_stale = to_copy.m_is_pruned_nsq_stale;
                m_is_pruned_apr_stale = to_copy.m_is_pruned_apr_stale;
            }
            else
            {
                m_parent_search       = &to_copy;
                m_is_closed_apr_stale = true;
                m_is_closed_nsq_stale = true;
                m_is_pruned_apr_stale = true;
                m_is_pruned_nsq_stale = true;
            }
        }

        /**!
         * \brief  Factory that returns a deep copy of a ditags object
         *
         * @param parameters should the copy be a deep copy
         */
        DitagsTetaq getDeepCopy()
        {
            return DitagsTetaq(*this, true);
        }

        /**!
         * \brief  Factory that returns a deep copy of a ditags object
         *
         * @param parameters should the copy be a deep copy
         */
        DitagsTetaq getShallowCopy()
        {
            return DitagsTetaq(*this, false);
        }

       protected:
        /**!
         * \brief Deep Copy Constructor
         *
         * @param parameters input problems
         */
        bool needToUpdateClosed(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const override
        {
            return needToUpdateClosedAPR(old_problem_inputs);
        }

        /**!
         * \brief Deep Copy Constructor
         *
         * @param parameters input problems
         */
        bool needToUpdatePruned(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const override
        {
            return needToUpdatePrunedAPR(old_problem_inputs);
        }

        /**!
         * \brief Deep Copy Constructor
         *
         * \return
         */
        bool needToUpdateOpen(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const override
        {
            bool motion_planning_changed = motionPlansNeedUpdate(old_problem_inputs);

            return (needToUpdateOpenAPR(old_problem_inputs) || needToUpdateOpenNSQ(old_problem_inputs) ||
                    motion_planning_changed);
        }

        /**!
         * \brief  Update the nodes in the closed set
         *
         */
        void updateClosed(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) override
        {
            updateNodesClosedAPR();
        }

        /**!
         * \brief  Update the nodes in the pruned set
         *
         */
        void updatePruned(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) override
        {
            updateNodesPrunedAPR();
        }

        /**!
         * \brief Update the nodes in the open set
         *
         */
        void updateOpen(const std::shared_ptr<const ItagsProblemInputs> &old_inputs) override
        {
            bool motion_planning_changed = motionPlansNeedUpdate(old_inputs);  // updates motion planner map

            if(needToUpdateOpenAPR(old_inputs))  // Traits changed in any way
            {
                updateNodesOpenAPR();
                m_is_closed_apr_stale = false;
                m_is_pruned_apr_stale = false;
            }
            if(needToUpdateOpenNSQ(old_inputs) || motion_planning_changed)  // Lo Task/Lost Agent or Obs region change
                                                                            // or Task Duration change/Precedence Change
            {
                updateNodesOpenNSQ(old_inputs);
                m_is_closed_nsq_stale = false;
                m_is_pruned_nsq_stale = false;
            }
        }

        /**!
         * \brief Update the NSQ value of a node
         *
         * \param a node needs its NSQ values updated
         */
        void updateNodeNSQ(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
        {
            std::dynamic_pointer_cast<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                m_heuristic)
                ->getNSQ(node);
            recomputeTETAQLocal(node);
        }

        /**!
         * \brief Update the APR value of a node
         *
         * \param a node needs its APR values updated
         */
        void updateNodeAPR(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
        {
            std::dynamic_pointer_cast<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                m_heuristic)
                ->getAPR(node);
            recomputeTETAQLocal(node);
        }

        /**!
         * \brief Update the h value of a node using the nodes internal APR and NSQ
         * without calling scheduling or recomputing APR or NSQ
         *
         *
         */
        void recomputeTETAQLocal(const std::shared_ptr<DynIncrementalTaskAllocationNode> &node)
        {
            float alpha =
                std::dynamic_pointer_cast<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                    m_heuristic)
                    ->getAlpha();
            if(!node->getNSQ().has_value())
            {
                updateNodeNSQ(node);
            }
            if(!node->getAPR().has_value())
            {
                updateNodeAPR(node);
            }
            float tetaq = alpha * node->getAPR().value() + (1.0f - alpha) * node->getNSQ().value();
            node->setH(tetaq);
        }

        /**!
         * \brief Function compares a passed problem_input to the current problem input and checks if the closed set NSQ
         * needs correcting
         *
         * \param const std::shared_ptr<const ItagsProblemInputs>& the old problem inputs to be compared to
         */
        [[nodiscard]] bool needToUpdateClosedAPR(
            const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const
        {
            if(old_problem_inputs->desiredTraitsMatrix().rows() < m_problem_inputs->desiredTraitsMatrix().rows())
            {
                return false;
            }
            const Eigen::MatrixXf &old_reqs = old_problem_inputs->desiredTraitsMatrix();
            const Eigen::MatrixXf &new_reqs = m_problem_inputs->desiredTraitsMatrix();
            for(int i = 0; i < old_reqs.rows(); ++i)
            {
                for(int j = 0; j < old_reqs.cols(); ++j)
                {
                    if(old_reqs.coeff(i, j) > new_reqs.coeff(i, j))
                    {
                        return true;
                    }
                }
            }
            const Eigen::MatrixXf &old_traits = old_problem_inputs->teamTraitsMatrix();
            const Eigen::MatrixXf &new_traits = m_problem_inputs->teamTraitsMatrix();
            for(int i = 0; i < old_traits.rows(); ++i)
            {
                for(int j = 0; j < old_traits.cols(); ++j)
                {
                    if(old_traits.coeff(i, j) < new_traits.coeff(i, j))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /**!
         * \brief Update the APR value of the Closed set nodes and add them to the open set
         *
         */
        void updateNodesClosedAPR()
        {
            // Update the closed set nodes if object is a deep copy
            if(m_is_deep_copy)
            {
                // update the apr for the closed set
                // Loop through the closed set and check if any are goals if so add them to the open set
                for(int i = m_closed.size() - 1; i >= 0; --i)
                {
                    std::shared_ptr<DynIncrementalTaskAllocationNode> node = m_closed[i];

                    if(m_goal_check->operator()(node))
                    {
                        updateNodeAPR(node);
                        // Remove from closed set
                        m_closed_ids.erase(m_closed[i]->hash());
                        m_closed.erase(m_closed.begin() + i);
                        // Add to open set
                        m_open.push(node->id(), node);
                    }
                }
            }
            // Update the closed set nodes if object is shallow copy
            else
            {
                std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> parent_closed;
                const DitagsTetaq *parent = this;
                while(parent != nullptr)
                {
                    // Look through the closed set of yourself and your parents and if any are goals
                    // add them to your closed set
                    for(int i = parent->m_closed.size() - 1; i >= 0; --i)
                    {
                        std::shared_ptr<DynIncrementalTaskAllocationNode> node = parent->m_closed[i];
                        if(m_goal_check->operator()(node))
                        {
                            std::shared_ptr<DynIncrementalTaskAllocationNode> open_node;
                            if(node->parent() == NULL)
                            {
                                open_node =
                                    std::make_shared<DynIncrementalTaskAllocationNode>(node->matrixDimensions());
                            }
                            else
                            {
                                open_node =
                                    std::make_shared<DynIncrementalTaskAllocationNode>(node->lastAssigment().value(),
                                                                                       node->parent());
                            }
                            open_node->setStatus(SearchNodeStatus::e_open);
                            open_node->setId(node->id());
                            open_node->setIsDynamicRoot(true);
                            open_node->setAPR(0);
                            if(m_is_closed_nsq_stale)
                            {
                                open_node->setNSQ(
                                    std::dynamic_pointer_cast<
                                        const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                                        m_heuristic)
                                        ->getNSQ(node));
                            }
                            else
                            {
                                open_node->setNSQ(node->getNSQ());
                            }
                            recomputeTETAQLocal(node);
                            m_open.push(node->hash(), open_node);
                            m_closed_ids.erase(node->hash());
                            if(this == parent)
                            {
                                m_closed.erase(m_closed.begin() + i);
                            }
                        }
                    }
                    // Move on to your parent search
                    parent = parent->m_parent_search;
                }
            }
        }

        /**!
         * \brief Function compares a passed problem_input to the current problem input and checks if the pruned set NSQ
         * needs correcting
         *
         * \param const std::shared_ptr<const ItagsProblemInputs>& the old problem inputs to be compared to
         */
        [[nodiscard]] bool needToUpdatePrunedAPR(
            const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const
        {
            // traits went up
            const Eigen::MatrixXf &old_reqs = old_problem_inputs->desiredTraitsMatrix();
            const Eigen::MatrixXf &new_reqs = m_problem_inputs->desiredTraitsMatrix();
            for(int i = 0; i < old_reqs.rows(); ++i)
            {
                for(int j = 0; j < old_reqs.cols(); ++j)
                {
                    if(old_reqs(i, j) < new_reqs(i, j))
                    {
                        return true;
                    }
                }
            }
            const Eigen::MatrixXf &old_traits = old_problem_inputs->teamTraitsMatrix();
            const Eigen::MatrixXf &new_traits = m_problem_inputs->teamTraitsMatrix();
            for(int i = 0; i < old_traits.rows(); ++i)
            {
                for(int j = 0; j < old_traits.cols(); ++j)
                {
                    if((old_traits.coeff(i, j) > new_traits.coeff(i, j)) ||
                       (old_traits.coeff(i, j) == 0 && new_traits.coeff(i, j) > 0))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /**!
         * \brief Update the APR value of the Pruned set nodes and add them to the open set
         *
         */
        void updateNodesPrunedAPR()
        {
            // Update the Pruned set nodes if object is a deep copy
            if(m_is_deep_copy)
            {
                // update the apr for the pruned set
                // Loop through the pruned set and check if any are goals if so add them to the open set
                for(int i = m_pruned.size() - 1; i >= 0; --i)
                {
                    std::shared_ptr<DynIncrementalTaskAllocationNode> node = m_pruned[i];
                    if(m_goal_check->operator()(node))
                    {
                        updateNodeAPR(node);
                        // Remove from pruned set
                        node->setAPR(0);
                        m_pruned_ids.erase(m_pruned[i]->hash());
                        m_pruned.erase(m_pruned.begin() + i);
                        // Add to open set
                        m_open.push(node->id(), node);
                    }
                }
            }
            // Update the pruned set nodes if object is shallow copy
            else
            {
                std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> parent_closed;
                const DitagsTetaq *parent = this;
                while(parent != nullptr)
                {
                    // Look through the pruned set of yourself and your parents and if any are goals
                    // add them to your pruned set
                    for(int i = parent->m_pruned.size() - 1; i >= 0; --i)
                    {
                        std::shared_ptr<DynIncrementalTaskAllocationNode> node = parent->m_pruned[i];

                        if(m_goal_check->operator()(node))
                        {
                            std::shared_ptr<DynIncrementalTaskAllocationNode> open_node;
                            if(node->parent() == NULL)
                            {
                                open_node =
                                    std::make_shared<DynIncrementalTaskAllocationNode>(node->matrixDimensions());
                            }
                            else
                            {
                                open_node =
                                    std::make_shared<DynIncrementalTaskAllocationNode>(node->lastAssigment().value(),
                                                                                       node->parent());
                            }
                            open_node->setStatus(SearchNodeStatus::e_open);
                            open_node->setId(node->id());
                            open_node->setIsDynamicRoot(true);
                            open_node->setAPR(0);
                            if(m_is_pruned_nsq_stale)
                            {
                                open_node->setNSQ(
                                    std::dynamic_pointer_cast<
                                        const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                                        m_heuristic)
                                        ->getNSQ(node));
                            }
                            else
                            {
                                open_node->setNSQ(node->getNSQ());
                            }
                            recomputeTETAQLocal(node);
                            m_open.push(node->hash(), open_node);
                            m_pruned_ids.erase(node->hash());
                            if(this == parent)
                            {
                                m_pruned.erase(m_pruned.begin() + i);
                            }
                        }
                    }
                    // Move on to your parent search
                    parent = parent->m_parent_search;
                }
            }
        }

        /**!
         * \brief Function compares a passed problem_input to the current problem input and checks if the open set APR
         * needs correcting
         *
         * \param const std::shared_ptr<const ItagsProblemInputs>& the old problem inputs to be compared to
         */
        [[nodiscard]] bool needToUpdateOpenAPR(
            const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const
        {
            // bool new_agent_added = old_problem_inputs->numberOfRobots() < this->m_problem_inputs->numberOfRobots();
            if(old_problem_inputs->numberOfPlanTasks() < m_problem_inputs->numberOfPlanTasks())
            {
                return true;
            }

            if(needToUpdateClosedAPR(old_problem_inputs) || needToUpdatePrunedAPR(old_problem_inputs))
            {
                return true;
            }
            return false;
        }

        /**!
         * \brief Update the APR value of the open set nodes
         *
         */
        void updateNodesOpenAPR()
        {
            // for each node in the open set update its apr value
            MutablePriorityQueue<unsigned int, float, DynIncrementalTaskAllocationNode> open_new;
            for(auto it = m_open.begin(); it != m_open.end(); it++)
            {
                std::shared_ptr<DynIncrementalTaskAllocationNode> node = (it->payload());
                updateNodeAPR(node);
                open_new.push(m_memoization->operator()(node), node);
            }
            m_open = open_new;
        }

        /**!
         * \brief Function compares a passed problem_input to the current problem input and checks if the open set NSQ
         * needs correcting
         *
         * \param const std::shared_ptr<const ItagsProblemInputs>& the old problem inputs to be compared to
         */
        [[nodiscard]] bool needToUpdateOpenNSQ(
            const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const
        {
            if(old_problem_inputs->scheduleWorstMakespan() !=
                   m_problem_inputs->scheduleWorstMakespan() ||  // sigma_worst changed
               old_problem_inputs->scheduleBestMakespan() !=
                   m_problem_inputs->scheduleBestMakespan() ||  // sigma_best changed
               old_problem_inputs->planTaskDurations() != m_problem_inputs->planTaskDurations() ||
               // durations changed
               old_problem_inputs->precedenceConstraints() !=
                   m_problem_inputs->precedenceConstraints() ||  // precedence constraints changed
               motionPlansNeedUpdate(old_problem_inputs))
            {
                return true;
            }
            return false;
        }

        /**!
         * \brief Update the NSQ values of the open set nodes
         *
         */
        void updateNodesOpenNSQ(const std::shared_ptr<const ItagsProblemInputs> &old_inputs)
        {
            auto scheduler_problem_inputs =
                std::make_shared<SchedulerProblemInputs>(m_problem_inputs,
                                                         m_open.begin()->payload()->allocation(),
                                                         std::set<std::pair<unsigned int, unsigned int>>());
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            scheduler.recomputEnviroment();
            for(auto it = m_open.begin(); it != m_open.end(); ++it)
            {
                if(it->payload()->m_schedule != NULL)
                {
                    if(!canSimpleRepair(it->payload(), old_inputs))
                    {
                        updateNodeNSQ(it->payload());
                    }
                }
            }
        }

        /**!
         * \brief Update the NSQ values of the open set nodes
         *
         */
        void updateNodesOpenNSQ()
        {
            auto scheduler_problem_inputs =
                std::make_shared<SchedulerProblemInputs>(m_problem_inputs,
                                                         m_open.begin()->payload()->allocation(),
                                                         std::set<std::pair<unsigned int, unsigned int>>());
            DeterministicMilpScheduler scheduler(scheduler_problem_inputs);
            scheduler.recomputEnviroment();
            for(auto it = m_open.begin(); it != m_open.end(); ++it)
            {
                updateNodeNSQ(it->payload());
            }
        }

        /**!
         * \brief Add the old solution to the open set
         *
         */
        void addPreviousSolutionToOpen() override
        {
            if(m_is_deep_copy)
            {
                std::shared_ptr<DynIncrementalTaskAllocationNode> node = m_closed.back();
                updateNodeNSQ(node);
                m_open.push(m_memoization->operator()(node), node);
                m_closed_ids.erase(m_memoization->operator()(node));
                m_closed.erase(m_closed.begin() + m_closed.size());
            }
            else
            {
                const DitagsTetaq *parent = this;
                while(parent != nullptr)
                {
                    if(parent->m_closed.size() > 0)
                    {
                        std::shared_ptr<DynIncrementalTaskAllocationNode> node = parent->m_closed.back();
                        m_open.push(m_memoization->operator()(node), node);
                        if(parent == this)
                        {
                            m_closed_ids.erase(m_memoization->operator()(node));
                            // m_closed.erase(m_closed.begin() + m_closed.size());
                        }
                        return;
                    }
                    parent = parent->m_parent_search;
                }
            }
        }

        /**!
         * \brief Checks to see if simple Schedule repair is possible and if it is it updates it
         *
         * \param returns returns a bool if the simple update was sucessful
         */
        std::vector<std::pair<float, float>> newTaskAdjustAmount(
            std::shared_ptr<DynIncrementalTaskAllocationNode> node,
            const std::shared_ptr<const ItagsProblemInputs> &old_inputs,
            const std::vector<std::pair<float, float>> time_points,
            const std::vector<std::pair<unsigned int, unsigned int>> mutex,
            const std::multimap<unsigned int, unsigned int> prec,
            float makespan)
        {
            std::vector<std::pair<float, float>> time_points_non = time_points;
            if(old_inputs->numberOfPlanTasks() < m_problem_inputs->numberOfPlanTasks())
            {
                int num_new = m_problem_inputs->numberOfPlanTasks() - old_inputs->numberOfPlanTasks();
                for(unsigned int i = 0; i < num_new; ++i)
                {
                    for(unsigned int j = 0; j < time_points.size(); ++j)
                    {
                        if(time_points[j].second + m_problem_inputs->planTaskDurations()[time_points.size() - i] >
                           makespan)
                        {
                            auto result = prec.equal_range('c');
                            for(auto it = result.first; it != result.second; it++)
                            {
                                if(it->second == (time_points.size() - i))
                                    time_points_non[time_points.size() - i].first = time_points[j].second;
                                time_points_non[time_points.size() - i].first =
                                    time_points[time_points.size() - i].first +
                                    m_problem_inputs->planTaskDurations()[time_points.size() - i];
                            }
                        }
                    }
                }
            }
            return time_points_non;
        }

        /**!
         * \brief Tries to simply update the schedule for the loss of a task
         *
         * \param returns a task_times object which is a vector of the start and end times of the task
         */
        std::vector<std::pair<float, float>> DurUpAdjustAmount(
            std::shared_ptr<DynIncrementalTaskAllocationNode> node,
            const std::shared_ptr<const ItagsProblemInputs> &old_inputs,
            const std::vector<std::pair<float, float>> time_points,
            const std::vector<std::pair<unsigned int, unsigned int>> mutex,
            const std::multimap<unsigned int, unsigned int> prec,
            float makespan)
        {
            std::vector<std::pair<float, float>> time_points_non = time_points;
            for(unsigned int i = 0; i < m_problem_inputs->numberOfPlanTasks(); ++i)
            {
                if(i < old_inputs->numberOfPlanTasks() &&
                   m_problem_inputs->planTask(i)->staticDuration() > old_inputs->planTask(i)->staticDuration())
                {
                    std::vector<unsigned int> precedented = {i};
                    float added = m_problem_inputs->planTaskDurations()[i] - old_inputs->planTaskDurations()[i];
                    time_points_non[i].second += added;
                    while(!precedented.empty())
                    {
                        int current_task = precedented[0];
                        precedented.erase(precedented.begin());
                        auto result = prec.equal_range(current_task);
                        for(auto it = result.first; it != result.second; it++)
                        {
                            if(time_points_non[it->first].second < time_points_non[it->second].first)
                            {
                                float duration = time_points_non[it->second].second - time_points_non[it->second].first;
                                time_points_non[it->second].first =
                                    m_problem_inputs->planTaskDurations()[time_points_non[it->first].second];
                                time_points_non[current_task].second = time_points_non[it->second].first + duration;
                                if(time_points_non[it->second].second > makespan)
                                {
                                    std::vector<std::pair<float, float>> empty;
                                    return empty;
                                }
                                precedented.push_back(it->second);
                            }
                        }
                    }

                    precedented = {i};
                    while(!precedented.empty())
                    {
                        int current_task = precedented[0];
                        precedented.erase(precedented.begin());
                        for(int j = 0; j < mutex.size(); ++j)
                        {
                            if(mutex[j].first == current_task &&
                               time_points_non[current_task].second > time_points_non[mutex[j].second].first)
                            {
                                float dur =
                                    time_points_non[mutex[j].second].second - time_points_non[mutex[j].second].first;
                                time_points_non[mutex[j].second].first  = time_points_non[current_task].second;
                                time_points_non[mutex[j].second].second = time_points_non[mutex[j].second].first + dur;
                                precedented.push_back(mutex[mutex[j].second].second);
                                if(time_points_non[mutex[j].second].second > makespan)
                                {
                                    std::vector<std::pair<float, float>> empty;
                                    return empty;
                                }
                            }
                        }
                    }
                }
            }
            return time_points_non;
        }

        /**!
         * \brief Tries to simply update the schedule for the loss of a task
         *
         * \param returns a task_times object which is a vector of the start and end times of the task
         */
        std::vector<std::pair<float, float>> lostTaskAdjustAmount(
            std::shared_ptr<DynIncrementalTaskAllocationNode> node,
            const std::shared_ptr<const ItagsProblemInputs> &old_inputs,
            const std::vector<std::pair<float, float>> time_points,
            const std::vector<std::pair<unsigned int, unsigned int>> mutex,
            const std::multimap<unsigned int, unsigned int> prec,
            float makespan)
        {
            std::vector<std::pair<float, float>> time_points_non = time_points;
            float last_time                                      = 0;
            std::vector<int> last_task                           = {};
            std::vector<int> lost_task                           = {};
            for(int i = 0; i < m_problem_inputs->numberOfPlanTasks(); ++i)
            {
                if(i < old_inputs->numberOfPlanTasks() && m_problem_inputs->planTask(i)->name() == "lostTask" &&
                   old_inputs->planTask(i)->name() != "lostTask")
                {
                    lost_task.push_back(i);
                }
                if(time_points[i].second > last_time)
                {
                    last_time = time_points[i].second;
                    last_task = {i};
                }
                else if(time_points[i].second == last_time)
                {
                    last_task.push_back(i);
                }
            }

            std::vector<int> precedented = lost_task;
            while(!precedented.empty())
            {
                int current_task = precedented[0];
                precedented.erase(precedented.begin());
                for(int i = 0; i < last_task.size(); ++i)
                {
                    for(int j = 0; j < mutex.size(); ++j)
                    {
                        if(mutex[j].first == current_task)
                        {
                            if(mutex[j].second == last_task[i])
                            {
                                std::vector<std::pair<float, float>> empty;
                                return empty;
                            }
                            precedented.push_back(mutex[j].second);
                        }
                    }
                }
                auto result = prec.equal_range(current_task);
                for(auto it = result.first; it != result.second; it++)
                {
                    for(int i = 0; i < last_task.size(); ++i)
                    {
                        if(it->second == current_task)
                        {
                            if(it->second == last_task[i])
                            {
                                std::vector<std::pair<float, float>> empty;
                                return empty;
                            }
                            precedented.push_back(it->second);
                        }
                    }
                }
            }
            return time_points_non;
        }

        /**!
         * \brief Tries to simply update the schedule for the loss of a task
         *
         * \param returns a task_times object which is a vector of the start and end times of the task
         */
        std::vector<std::pair<float, float>> DurDownAdjustAmount(
            std::shared_ptr<DynIncrementalTaskAllocationNode> node,
            const std::shared_ptr<const ItagsProblemInputs> &old_inputs,
            const std::vector<std::pair<float, float>> time_points,
            const std::vector<std::pair<unsigned int, unsigned int>> mutex,
            const std::multimap<unsigned int, unsigned int> prec,
            float makespan)
        {
            std::vector<std::pair<float, float>> time_points_non = time_points;
            float last_time                                      = 0;
            std::vector<int> last_task                           = {};
            std::vector<int> dur_down_tasks                      = {};
            for(int i = 0; i < m_problem_inputs->numberOfPlanTasks(); ++i)
            {
                if(m_problem_inputs->planTask(i)->staticDuration() < m_problem_inputs->planTask(i)->staticDuration())
                {
                    dur_down_tasks.push_back(i);
                }
                if(time_points[i].second > last_time)
                {
                    last_time = time_points[i].second;
                    last_task = {i};
                }
                else if(time_points[i].second == last_time)
                {
                    last_task.push_back(i);
                }
            }

            std::vector<int> precedented = dur_down_tasks;
            while(!precedented.empty())
            {
                int current_task = precedented[0];
                precedented.erase(precedented.begin());
                for(int i = 0; i < last_task.size(); ++i)
                {
                    for(int j = 0; j < mutex.size(); ++j)
                    {
                        if(mutex[j].first == current_task)
                        {
                            if(mutex[j].second == last_task[i])
                            {
                                std::vector<std::pair<float, float>> empty;
                                return empty;
                            }
                            precedented.push_back(mutex[j].second);
                        }
                    }
                }
                auto result = prec.equal_range(current_task);
                for(auto it = result.first; it != result.second; it++)
                {
                    for(int i = 0; i < last_task.size(); ++i)
                    {
                        if(it->second == current_task)
                        {
                            if(it->second == last_task[i])
                            {
                                std::vector<std::pair<float, float>> empty;
                                return empty;
                            }
                            precedented.push_back(it->second);
                        }
                    }
                }
            }
            return time_points_non;
        }

        /**!
         * \brief Checks to see if simple Schedule repair is possible and if it is it updates it
         *
         * \param returns returns a bool if the simple update was sucessful
         */
        bool canSimpleRepair(const std::shared_ptr<DynIncrementalTaskAllocationNode> node,
                             const std::shared_ptr<const ItagsProblemInputs> &old_inputs)
        {
            if(node->getAPR() == 0)
            {
                return false;
            }
            float makespan = node->schedule()->makespan();
            std::vector<std::pair<float, float>> time_points =
                std::dynamic_pointer_cast<const DeterministicSchedule>(node->m_schedule)->timepoints();
            const std::vector<std::pair<unsigned int, unsigned int>> mutex =
                std::dynamic_pointer_cast<const DeterministicSchedule>(node->m_schedule)
                    ->precedenceSetMutexConstraints();
            const std::multimap<unsigned int, unsigned int> prec = m_problem_inputs->precedenceConstraints();
            time_points = newTaskAdjustAmount(node, old_inputs, time_points, mutex, prec, makespan);
            for(int i = 0; i < time_points.size(); ++i)
            {
                if(time_points[i].second > makespan)
                {
                    makespan = time_points[i].second;
                }
            }
            time_points = DurUpAdjustAmount(node, old_inputs, time_points, mutex, prec, makespan);
            if(time_points.empty())
            {
                return false;
            }
            time_points = lostTaskAdjustAmount(node, old_inputs, time_points, mutex, prec, makespan);
            if(time_points.empty())
            {
                return false;
            }
            time_points = DurDownAdjustAmount(node, old_inputs, time_points, mutex, prec, makespan);
            if(time_points.empty())
            {
                return false;
            }

            float last = 0;
            for(int i = 0; i < time_points.size(); ++i)
            {
                if(time_points[i].second > last)
                {
                    last = time_points[i].second;
                }
            }
            node->setNSQ((m_problem_inputs->scheduleWorstMakespan() - last) /
                         (m_problem_inputs->scheduleWorstMakespan() - m_problem_inputs->scheduleBestMakespan()));
            recomputeTETAQLocal(node);
            return true;
        }

        bool m_is_closed_nsq_stale;  // are the nsq values in the closed set up to date
        bool m_is_closed_apr_stale;  // are the apr values in the closed set up to date
        bool m_is_pruned_nsq_stale;  // are the nsq values in the pruned set up to date
        bool m_is_pruned_apr_stale;  // are the apr values in the pruned set up to date

       private:
        const DitagsTetaq *m_parent_search;
    };

}  // namespace grstapse

// Not Finished
// 8

// Repair Comments
// Functions to implement
// 1) Add old solution to open set -Done
// 2) Update Open set Apr
// 3) Mark closed/pruned set APR schedules as stale -Done
// 4) Closed Set APR Update and move solutions to Open Set
// 5) Add new nodes from root that have new agent added to each task
// 6) Update Open set NSQ -Done
// 7) Mark closed/prune set NSQ as stale -Done
// 8) Update map remove edges that are now invalid update open set
// 9)If NSQ stale update NSQ of closed set nodes moved -Done

// Repair Methods

//  Trait Down/Reqs Up
//// 1)Add old solution to open set
//// 2)Open set APR update
//// 3)Mark closed/pruned apr as stale

// Traits Up/Reqs Down
//// 2)Open Set APR Update
//// 4)Closed Set APR Update and move solutions to Open Set
//// 9)If NSQ stale update NSQ of closed set nodes moved

// New Agent
//// 1)Add old solution to open set
//// 5)Add new nodes from root that have new agent added to each task

// Lost Task
//// 2)Open Set APR Update
//// 4)Closed Set APR Update and move solutions to Open Set
//// 6)Update NSQ for open set
//// 7)Mark the Closed/Pruned set NSQ as stale

// New Task/Lost Agent
//// 1)Add old solution to open set
//// 2)Update Open set Apr
//// 6)Update Open set NSQ
//// 7)Mark closed/pruned set NSQ schedules as stale
//// 3)Mark closed/pruned set APR schedules as stale

// Task Duration change/Precedence Change
//// 1)Add old solution to open set
//// 6)Update Open set NSQ
//// 7)Mark closed/pruned set NSQ schedules as stale

// Obs region change
//// 8)Update map remove edges that are now invalid update open set
//// 1)Add old solution to open set
//// 6)Update Open set NSQ
//// 7)Mark closed/prune set schedules as stale

// Complete Repair System
// 0) Check if map needs updating and if so update it (8)
////1) if step 0 mark closed/prune set schedules as stale (7)
// 2)Add old solution to open set  (1)
// 3)Check if open set needs APR update (2)
////4)Perform Needed APR Updates to open set
// 5)Check if closed set needs APR update (4)
////6)Perform Needed APR Updates to closed set and check for solutions if found add to open (4)
////7)
////8)If NSQ stale update NSQ of closed set nodes moved(9)
// 8)Mark closed/prune set apr stale if required (3)
// 9)Check if open set NSQ needs to be updated
/// 10)Update Open set NSQ (6)
// 11) Check if new nodes from root need to be added and add if needed (5)
