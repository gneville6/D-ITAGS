#include "mock_ditags.hpp"

namespace grstapse::mocks
{
    MockDitags::MockDitags(const std::shared_ptr<const ItagsProblemInputs>& problem_inputs)
        : Base{problem_inputs}
    {}

    MockDitags::MockDitags(const DitagsTetaq& to_copy, bool is_deep_copy)
        : Base(to_copy, is_deep_copy)
    {}

    MutablePriorityQueue<unsigned int, float, DynIncrementalTaskAllocationNode>& MockDitags::getOpen()
    {
        return m_open;
    }

    std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>>& MockDitags::getClosed()
    {
        return m_closed;
    }

    std::set<unsigned int>& MockDitags::getClosedID()
    {
        return m_closed_ids;
    }

    std::vector<std::shared_ptr<DynIncrementalTaskAllocationNode>>& MockDitags::getPruned()
    {
        return m_pruned;
    }

    std::set<unsigned int>& MockDitags::getPrunedID()
    {
        return m_pruned_ids;
    }

    std::shared_ptr<const ItagsProblemInputs>& MockDitags::getItagsProblemInputs()
    {
        return m_problem_inputs;
    }

    void MockDitags::addNodeToOpen(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
    {
        const unsigned int id = m_memoization->operator()(node);
        m_open.push(id, node);
    }

    void MockDitags::addNodeToClosed(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
    {
        const unsigned int id = m_memoization->operator()(node);
        m_closed.push_back(node);
        m_closed_ids.insert(id);
    }

    void MockDitags::addNodeToPruned(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
    {
        const unsigned int id = m_memoization->operator()(node);
        m_pruned.push_back(node);
        m_pruned_ids.insert(id);
    }

    float MockDitags::recalcAPR(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
    {
        return std::dynamic_pointer_cast<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                   this->m_heuristic)
            ->getAPR(node);
    }

    float MockDitags::recalcNSQ(std::shared_ptr<DynIncrementalTaskAllocationNode> node)
    {
        return std::dynamic_pointer_cast<const TimeExtendedTaskAllocationQuality<DynIncrementalTaskAllocationNode>>(
                   this->m_heuristic)
            ->getNSQ(node);
    }

    std::shared_ptr<DynIncrementalTaskAllocationNode> MockDitags::popOpen()
    {
        return m_open.pop();
    }

}  // namespace grstapse::mocks