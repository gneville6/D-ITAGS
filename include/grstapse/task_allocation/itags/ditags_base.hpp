#pragma once

// Global
#include <memory>
#include <stack>

#include <unordered_map>
#include <unordered_set>

// Local Common
#include "grstapse/common/search/greedy_best_first_search/greedy_best_first_search.hpp"
#include "grstapse/common/search/hash_memoization.hpp"
#include "grstapse/common/search/search_statistics_common.hpp"
#include "grstapse/common/utilities/matrix_dimensions.hpp"
// Local Problem files
#include "grstapse/grstaps_problem_inputs.hpp"
#include "grstapse/task_planning/sas/sas_action.hpp"
// Local ITAGS Stuff
#include "grstapse/common/utilities/timer.hpp"
#include "grstapse/common/utilities/timer_runner.hpp"
#include "grstapse/robot.hpp"
#include "grstapse/species.hpp"
#include "grstapse/task_allocation/itags/desired_traits_check.hpp"
#include "grstapse/task_allocation/itags/ditags_problem_input_change.hpp"
#include "grstapse/task_allocation/itags/dyn_incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/incremental_allocation_generator.hpp"
#include "grstapse/task_allocation/itags/incremental_task_allocation_node.hpp"
#include "grstapse/task_allocation/itags/itags.hpp"
#include "grstapse/task_allocation/itags/itags_problem_inputs.hpp"
#include "grstapse/task_allocation/itags/task_allocation_math.hpp"
#include "grstapse/task_allocation/itags/time_extended_task_allocation_quality.hpp"
#include "grstapse/task_allocation/itags/traits_improvement_pruning.hpp"

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
    class DitagsBase
        : public Itags<TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>,
                       DynIncrementalTaskAllocationNode>
    {
       public:
        using Base = Itags<TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>,
                           DynIncrementalTaskAllocationNode>;

        /**!
         * \brief Constructor
         *
         * \param parameters input problems
         */
        explicit DitagsBase(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs)
            : Base{problem_inputs}
            , m_is_deep_copy(true)
        {}

        /**!
         * \brief  Copy constructor
         *
         * @param parameters input problems
         */
        explicit DitagsBase(const DitagsBase &to_copy, bool is_deep_copy = true)
            : Base{to_copy.m_problem_inputs}
        {
            if(is_deep_copy)
            {
                makeDeepCopy(to_copy);
            }
            else
            {
                makeShallowCopy(to_copy);
            }
        }

        /**!
         * \brief Repair a search after some change of information
         *
         * @param parameters a repair object that describes how a search has changed
         */
        void repairSearch(const std::shared_ptr<const DtagsProblemInputsChanges> &problem_changes)
        {
            std::shared_ptr<const ItagsProblemInputs> old_inputs = m_problem_inputs;
            m_problem_inputs                                     = problem_changes->getNewInputs();

            std::shared_ptr<DynIncrementalTaskAllocationNode> root = getRoot();
            root->setDimensions(m_problem_inputs->numberOfRobots(), m_problem_inputs->numberOfPlanTasks());

            addPreviousSolutionToOpen();

            bool motion_planning_changed;
            if(problem_changes->getMotionPlanChanged())  // updates motion planner map
            {
                updateMotionPlanningMap();
            }
            // 3)Check if open set needs APR update (2)
            if(problem_changes->getNeedUpdateClosed())
            {
                updateClosed(old_inputs);
            }
            if(problem_changes->getNeedUpdatePruned())
            {
                updatePruned(old_inputs);
            }
            if(problem_changes->getNeedUpdateOpen())
            {
                updateOpen(old_inputs);
            }
            if(problem_changes->getLostAgent())  // if agent lost repair
            {
                updateForLostAgent(old_inputs);
            }
            if(problem_changes->getNewAgent())  // new agent
            {
                addNewNodesFromRoot(old_inputs);
            }
        }

        /**!
         * \brief Repair a search after some change of information
         *
         * @param parameters a new problem input object that will replace the old problem inputs
         */
        void repairSearch(const std::shared_ptr<const ItagsProblemInputs> &new_problem_inputs)
        {
            TimerRunner timer_runner(m_parameters->timer_name);
            std::shared_ptr<const ItagsProblemInputs> old_inputs = m_problem_inputs;
            updateFunctors(new_problem_inputs, old_inputs);

            std::shared_ptr<DynIncrementalTaskAllocationNode> root = getRoot();
            root->setDimensions(new_problem_inputs->numberOfRobots(), new_problem_inputs->numberOfPlanTasks());

            addPreviousSolutionToOpen();
            bool motion_planning_changed;
            if(wasAgentLost(old_inputs))  // if agent lost repair
            {
                updateForLostAgent(old_inputs);
            }
            if(motion_planning_changed = motionPlansNeedUpdate(old_inputs))  // updates motion planner map
            {
                updateMotionPlanningMap();
            }
            // 3)Check if open set needs APR update (2)
            if(needToUpdateClosed(old_inputs))
            {
                updateClosed(old_inputs);
            }
            if(needToUpdatePruned(old_inputs))
            {
                updatePruned(old_inputs);
            }
            if(needToUpdateOpen(old_inputs))
            {
                updateOpen(old_inputs);
            }
            if(wasNewAgentAdded(old_inputs))  // new agent
            {
                addNewNodesFromRoot(old_inputs);
            }
        }

       protected:
        /**!
         * \brief Returns the root node of the graph
         *
         */
        std::shared_ptr<DynIncrementalTaskAllocationNode> getRoot()
        {
            return m_root;
        }

        /**!
         * \brief CHecks to see if on repair the closed set needs to be updated
         *
         * @param parameters input problems
         */
        virtual bool needToUpdateClosed(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const = 0;

        /**!
         * \brief CHecks to see if on repair the pruned set needs to be updated
         *
         * @param parameters input problems
         */
        virtual bool needToUpdatePruned(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const = 0;

        /**!
         * \brief  CHecks to see if on repair the open set needs to be updated
         *
         * \return
         */
        virtual bool needToUpdateOpen(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) const = 0;

        /**!
         * \brief  Update the nodes in the closed set
         *
         */
        virtual void updateClosed(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) = 0;

        /**!
         * \brief  Update the nodes in the pruned set
         *
         */
        virtual void updatePruned(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) = 0;

        /**!
         * \brief Update the nodes in the open set
         *
         */
        virtual void updateOpen(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs) = 0;

        /**!
         * \brief Deep Copy Constructor
         *
         * \param parameters input problems
         */
        void makeDeepCopy(const DitagsBase &to_copy)
        {
            try
            {
                if(!to_copy.m_is_deep_copy)
                {
                    throw 505;
                }
            }
            catch(int e)
            {
                std::cout << "Cant Deep Copy a Shallow Object\" \n";
                return;
            }
            m_is_deep_copy = true;

            // Copy the problem inputs and non-open id vectors
            m_problem_inputs = to_copy.m_problem_inputs;
            m_closed_ids     = to_copy.m_closed_ids;
            m_pruned_ids     = to_copy.m_pruned_ids;

            // Create an unordered map that will be used to prevent re-creation of nodes
            // This unordered map is used to map nodes in the original search to their copies
            // in the new search
            std::unordered_map<std::shared_ptr<const DynIncrementalTaskAllocationNode>,
                               std::shared_ptr<DynIncrementalTaskAllocationNode>>
                previously_added_map;

            // Initialize a variable that will hold the current node that we are working on copying
            std::shared_ptr<const DynIncrementalTaskAllocationNode> current_node;
            // Loop through all the nodes in the open set of the original search
            // and add them and their ancestors to the copied search
            for(auto it = to_copy.m_open.begin(); it != to_copy.m_open.end(); ++it)
            {
                // This stack is used to create the branch that the node to copy is apart of
                std::stack<std::shared_ptr<const DynIncrementalTaskAllocationNode>> branch;
                // Add the current node to its branch
                branch.push((*it).payload());
                // Add the nodes ancestors to the branch until you reach the root node,
                // or you find a node already in the copied search
                std::shared_ptr<const DynIncrementalTaskAllocationNode> parent = (*it).payload()->parent();
                while(parent != nullptr && previously_added_map.find(parent) == previously_added_map.end())
                {
                    branch.push(parent);
                    parent = parent->parent();
                }
                // While the branch is not empty keep creating the nodes in the branch
                while(!branch.empty())
                {
                    // If only one node is in the branch that means it is the original open set node,
                    // and it should be copied and the copy added to the open set
                    if(branch.size() == 1)
                    {
                        // grab the node from the top of the branch and remove it
                        current_node = branch.top();
                        parent       = current_node->parent();
                        branch.pop();
                        // initialize the copy
                        std::shared_ptr<DynIncrementalTaskAllocationNode> open_node;
                        if(parent == NULL)
                        {
                            open_node =
                                std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions());
                            m_root = open_node;
                        }
                        else
                        {
                            open_node = std::make_shared<DynIncrementalTaskAllocationNode>(
                                current_node->lastAssigment().value(),
                                previously_added_map[parent]);
                        }

                        // set the members of the new copy
                        open_node->setStatus(SearchNodeStatus::e_closed);
                        open_node->setH(current_node->h());
                        open_node->setId(current_node->id());
                        open_node->setAPR(current_node->getAPR());
                        open_node->setNSQ(current_node->getNSQ());
                        open_node->m_schedule = current_node->m_schedule;
                        // add the new copy to the open set
                        previously_added_map[current_node] = open_node;
                        m_open.push(open_node->id(), open_node);
                    }
                    // Add this member of the branch to the closed set
                    else
                    {
                        // grab the node from the branch and remove it
                        current_node = branch.top();
                        branch.pop();
                        parent = current_node->parent();
                        // initialize the copy and add it to the closed set
                        if(parent == nullptr)
                        {
                            m_closed.push_back(
                                std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions()));
                            m_root = m_closed.back();
                        }
                        else
                        {
                            m_closed.push_back(std::make_shared<DynIncrementalTaskAllocationNode>(
                                current_node->lastAssigment().value(),
                                previously_added_map[parent]));
                        }

                        // set the members of the new copy
                        m_closed.back()->setStatus(SearchNodeStatus::e_pruned);
                        m_closed.back()->setH(current_node->h());
                        m_closed.back()->setId(current_node->id());
                        m_closed.back()->setAPR(current_node->getAPR());
                        m_closed.back()->setNSQ(current_node->getNSQ());
                        m_closed.back()->m_schedule        = current_node->m_schedule;
                        previously_added_map[current_node] = m_closed.back();
                    }
                }
            }
            // Loop through all the nodes in the pruned set of the original search
            // and add them and their ancestors to the copied search
            for(auto it = to_copy.m_pruned.begin(); it != to_copy.m_pruned.end(); ++it)
            {
                // This stack is used to create the branch that the node to copy is apart of
                std::stack<std::shared_ptr<const DynIncrementalTaskAllocationNode>> branch;
                // Add the current node to its branch
                branch.push(*it);
                // Add the nodes ancestors to the branch until you reach the root node,
                // or you find a node already in the copied search
                std::shared_ptr<const DynIncrementalTaskAllocationNode> parent = (*it)->parent();
                while(parent != nullptr && previously_added_map.find(parent) == previously_added_map.end())
                {
                    branch.push(parent);
                    parent = parent->parent();
                }
                // While the branch is not empty keep creating the nodes in the branch
                while(!branch.empty())
                {
                    // If only one node is in the branch that means it is the original open set node,
                    // and it should be copied and the copy added to the open set
                    if(branch.size() == 1)
                    {
                        // grab the node from the top of the branch and remove it
                        current_node = branch.top();
                        parent       = current_node->parent();
                        branch.pop();
                        // initialize the copy
                        if(parent == nullptr)
                        {
                            m_pruned.push_back(
                                std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions()));
                            m_root = m_pruned.back();
                        }
                        else
                        {
                            auto parental = previously_added_map[parent];
                            m_pruned.push_back(std::make_shared<DynIncrementalTaskAllocationNode>(
                                current_node->lastAssigment().value(),
                                previously_added_map[parent]));
                        }
                        // set the members of the new copy
                        m_pruned.back()->setStatus(SearchNodeStatus::e_pruned);
                        m_pruned.back()->setH(current_node->h());
                        m_pruned.back()->setId(current_node->id());
                        m_pruned.back()->setAPR(current_node->getAPR());
                        m_pruned.back()->setNSQ(current_node->getNSQ());
                        m_closed.back()->m_schedule        = current_node->m_schedule;
                        previously_added_map[current_node] = m_pruned.back();
                    }
                    else
                    {
                        // grab the node from the top of the branch and remove it
                        current_node = branch.top();
                        branch.pop();
                        parent = current_node->parent();
                        // initialize the copy
                        if(parent == nullptr)
                        {
                            m_closed.push_back(
                                std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions()));
                            m_root = m_closed.back();
                        }
                        else
                        {
                            m_closed.push_back(std::make_shared<DynIncrementalTaskAllocationNode>(
                                current_node->lastAssigment().value(),
                                previously_added_map[parent]));
                        }
                        // set the members of the new copy
                        m_closed.back()->setStatus(SearchNodeStatus::e_closed);
                        m_closed.back()->setH(current_node->h());
                        m_closed.back()->setId(current_node->id());
                        m_closed.back()->setAPR(current_node->getAPR());
                        m_closed.back()->setNSQ(current_node->getNSQ());
                        m_closed.back()->m_schedule        = current_node->m_schedule;
                        previously_added_map[current_node] = m_closed.back();
                    }
                }
            }

            // if there is a node unaccounted for it would be the solution from the last run
            if(to_copy.m_closed_ids.size() > this->m_closed.size())
            {
                // This stack is used to create the branch that the node to copy is apart of
                std::stack<std::shared_ptr<const DynIncrementalTaskAllocationNode>> branch;
                // Add the current node to its branch
                branch.push(to_copy.m_closed.back());
                // Add the nodes ancestors to the branch until you reach the root node,
                // or you find a node already in the copied search
                std::shared_ptr<const DynIncrementalTaskAllocationNode> parent = to_copy.m_closed.back()->parent();
                while(parent != nullptr && previously_added_map.find(parent) == previously_added_map.end())
                {
                    branch.push(parent);
                    parent = parent->parent();
                }
                // While the branch is not empty keep creating the nodes in the branch
                while(!branch.empty())
                {
                    // Add this member of the branch to the closed set
                    // grab the node from the branch and remove it
                    current_node = branch.top();
                    branch.pop();
                    parent = current_node->parent();
                    // initialize the copy and add it to the closed set
                    if(parent == nullptr)
                    {
                        m_closed.push_back(
                            std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions()));
                        m_root = m_closed.back();
                    }
                    else
                    {
                        m_closed.push_back(
                            std::make_shared<DynIncrementalTaskAllocationNode>(current_node->lastAssigment().value(),
                                                                               previously_added_map[parent]));
                    }

                    // set the members of the new copy
                    m_closed.back()->setStatus(SearchNodeStatus::e_pruned);
                    m_closed.back()->setH(current_node->h());
                    m_closed.back()->setId(current_node->id());
                    m_closed.back()->setAPR(current_node->getAPR());
                    m_closed.back()->setNSQ(current_node->getNSQ());
                    m_closed.back()->m_schedule        = current_node->m_schedule;
                    previously_added_map[current_node] = m_closed.back();
                }
            }

            // set the next id for future expansion
            m_open.begin()->payload()->setNextId(to_copy.m_open.begin()->payload()->nextId());
        }

        /**!
         * \brief Shallow Copy Constructor
         *
         * \param parameters input problems
         */
        void makeShallowCopy(const DitagsBase &to_copy)
        {
            m_is_deep_copy = false;
            m_root         = to_copy.m_root;
            // Copy the problem specification and the non-open ids
            m_problem_inputs = to_copy.m_problem_inputs;
            m_closed_ids     = to_copy.m_closed_ids;
            m_pruned_ids     = to_copy.m_pruned_ids;

            // Create some variables that we will use to help deep copy the open set
            std::shared_ptr<const DynIncrementalTaskAllocationNode> current_node;
            std::shared_ptr<const DynIncrementalTaskAllocationNode> parent;

            // Loop through the open set and copy all the open set nodes
            for(auto it = to_copy.m_open.begin(); it != to_copy.m_open.end(); ++it)
            {
                // Set the current node to copy and its parent
                parent       = (*it).payload()->parent();
                current_node = (*it).payload();
                // Create and initialize the node that will be the deep copy
                std::shared_ptr<DynIncrementalTaskAllocationNode> open_node;
                if(parent == nullptr)
                {
                    open_node = std::make_shared<DynIncrementalTaskAllocationNode>(current_node->matrixDimensions());
                }
                else
                {
                    open_node =
                        std::make_shared<DynIncrementalTaskAllocationNode>(current_node->lastAssigment().value(),
                                                                           parent);
                }

                // Set the internals of the new node to match the parents
                open_node->setStatus(SearchNodeStatus::e_open);
                open_node->setH(current_node->h());
                open_node->setId(current_node->id());
                open_node->setAPR(current_node->getAPR());
                open_node->setNSQ(current_node->getNSQ());
                open_node->m_schedule = current_node->m_schedule;
                // Set that this node to be a dynamic root
                // This means that this nodes parent is not in this graph
                open_node->setIsDynamicRoot(true);
                // Add nodes to open set
                m_open.push(open_node->id(), open_node);
            }
            // Set the next id of the search for future expansion
            current_node->setNextId(current_node->nextId());
        }

        /**!
         * \brief Update the functors for repair purposes
         *
         * \param parameters input problems
         */
        void updateFunctors(const std::shared_ptr<const ItagsProblemInputs> &problem_inputs,
                            const std::shared_ptr<const ItagsProblemInputs> &old_inputs)
        {
            m_problem_inputs = problem_inputs;
            m_heuristic = std::make_shared<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                m_problem_inputs);
            m_successor_generator =
                std::make_shared<const IncrementalAllocationGenerator<DynIncrementalTaskAllocationNode>>(
                    m_problem_inputs);
            m_goal_check =
                std::make_shared<const DesiredTraitsCheck<DynIncrementalTaskAllocationNode>>(m_problem_inputs);
            m_prepruning_method =
                std::make_shared<const TraitsImprovementPruning<DynIncrementalTaskAllocationNode>>(m_problem_inputs);
        }

        /**!
         * \brief Add the old solution to the open set
         *
         */
        virtual void addPreviousSolutionToOpen() = 0;

        /**!
         * \brief Was an agent lost when compared to the old problem inputs
         *
         * /param the old problem inputs
         *
         * /return a bool which
         *
         */
        [[nodiscard]] bool wasAgentLost(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs)
        {
            std::vector<std::shared_ptr<const Robot>> old_robots     = old_problem_inputs->robots();
            std::vector<std::shared_ptr<const Robot>> current_robots = m_problem_inputs->robots();
            for(int i = 0; i < old_robots.size(); ++i)
            {
                if(old_robots[i]->species()->name() != "lost_agent" &&
                   current_robots[i]->species()->name() == "lost_agent")
                {
                    return true;
                }
            }
            return false;
        }

        /**!
         * \brief Which agents were lost
         *
         * \param the old problem inputs
         *
         * \return a vector of ints which correspond to the agents that were lost
         */
        std::vector<int> agentsLost(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs)
        {
            std::vector<int> lostAgents;
            std::vector<std::shared_ptr<const Robot>> old_robots     = old_problem_inputs->robots();
            std::vector<std::shared_ptr<const Robot>> current_robots = m_problem_inputs->robots();
            for(int i = 0; i < old_robots.size(); ++i)
            {
                if(old_robots[i]->species()->name() != "lost_agent" &&
                   current_robots[i]->species()->name() == "lost_agent")
                {
                    lostAgents.push_back(i);
                }
            }
            return lostAgents;
        }

        /**!
         * \brief Update the open set of the graph for a lost agent by removing all nodes that used that agent
         *
         */
        void updateForLostAgent(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs)
        {
            std::vector<int> lost_agents        = agentsLost(old_problem_inputs);
            std::vector<unsigned int> lost_keys = {};
            for(auto it = m_open.begin(); it != m_open.end(); ++it)
            {
                if(shouldRemoveLostAgent(lost_agents, it->payload()))
                {
                    lost_keys.push_back(it->key());
                }
            }
            for(unsigned int key: lost_keys)
            {
                m_open.erase(key);
            }
        }

        /**!
         * \brief returns whether a node should be removed because it uses a lost agent
         *
         * /return bool whether this node should be removed or not
         */
        [[nodiscard]] bool shouldRemoveLostAgent(
            const std::vector<int> &lost_agents,
            const std::shared_ptr<const DynIncrementalTaskAllocationNode> &node) const
        {
            Eigen::MatrixXf allocation = node->allocation();
            for(int i: lost_agents)
            {
                for(int j = 0; j < allocation.rows(); ++j)
                {
                    if(allocation(j, i) == 1)  // if assigned remove this node and move on
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /**!
         * \brief Function compares a passed problem_input to the current problem input and checks if the new problem
         * has additional agents
         *
         * @param the old problem inputs to be compared to
         */
        [[nodiscard]] inline bool wasNewAgentAdded(const std::shared_ptr<const ItagsProblemInputs> &old_problem_inputs)
        {
            return old_problem_inputs->numberOfRobots() < Base::m_problem_inputs->numberOfRobots();
        }

        /**!
         * \brief Add new nodes from the root node because a new agent was created
         *
         */
        void addNewNodesFromRoot(std::shared_ptr<const ItagsProblemInputs> old_inputs)
        {
            // Create a copy of the root node for expansion purposes
            std::shared_ptr<DynIncrementalTaskAllocationNode> root = Base::createRootNode();
            for(int i = 0; i < m_closed.size(); ++i)
            {
                root = m_closed[i];
                // Add new children of root node to graph
                std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>> children =
                    m_successor_generator->operator()(root);
                m_statistics->incrementNodesExpanded();

                auto bfs_parameters = std::dynamic_pointer_cast<const BestFirstSearchParameters>(Base::m_parameters);

                const bool pruning_before_eval = m_prepruning_method != nullptr;

                for(const std::shared_ptr<DynIncrementalTaskAllocationNode> &child: children)
                {
                    const unsigned int id = m_memoization->operator()(child);

                    // Ignore if this node has already been closed or pruned
                    if(child->lastAssigment().value().robot <
                       (m_problem_inputs->numberOfRobots() -
                        (m_problem_inputs->numberOfRobots() - old_inputs->numberOfRobots())))
                    {
                        continue;
                    }

                    // Check if the child should be pruned before evaluation
                    if(pruning_before_eval && m_prepruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if(bfs_parameters->save_pruned_nodes)
                        {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Evaluate
                    Base::evaluateNode(child);
                    m_statistics->incrementNodesEvaluated();

                    // Check if child should be pruned after evaluation
                    if(!pruning_before_eval && m_prepruning_method->operator()(child))
                    {
                        child->setStatus(SearchNodeStatus::e_pruned);
                        m_statistics->incrementNodesPruned();
                        m_pruned_ids.insert(id);
                        if(bfs_parameters->save_pruned_nodes)
                        {
                            m_pruned.push_back(child);
                        }
                        continue;
                    }

                    // Add child to open set
                    child->setStatus(SearchNodeStatus::e_open);
                    m_open.push(id, child);
                }
            }
        }

        /**!
         * \brief Update the Map
         *
         */
        void updateMotionPlanningMap() {}

        /**!
         * \brief Would the passed problem inputs require updates
         *
         * \param Problem inputs to test if motion planning updates would be required
         *
         */
        bool motionPlansNeedUpdate(std::shared_ptr<const ItagsProblemInputs> old_inputs) const
        {
            return false;
        }

        // Member variables
        bool m_is_deep_copy;  //!< is this search a deep copy meaning all of its closed nodes are in the closed set
    };

}  // namespace grstapse
