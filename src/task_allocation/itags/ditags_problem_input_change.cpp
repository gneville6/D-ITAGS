#include "grstapse/task_allocation/itags/ditags_problem_input_change.hpp"

namespace grstapse
{
    /**!
     * Default Constructor for changes object
     */
    DtagsProblemInputsChanges::DtagsProblemInputsChanges()
        : m_repair_closed_needed(false)
        , m_repair_pruned_needed(false)
        , m_repair_open_needed(false)
        , m_mp_changed(false)
        , m_new_agent(false)
        , m_lost_agent(false)
    {}

    /**!
     * Constructor for changes object
     */
    DtagsProblemInputsChanges::DtagsProblemInputsChanges(std::shared_ptr<const ItagsProblemInputs> problem_inputs,
                                                         bool repair_open,
                                                         bool repair_closed,
                                                         bool repair_pruned,
                                                         bool mp_changed,
                                                         bool lost_agent,
                                                         bool new_agent)
        : m_new_inputs(problem_inputs)
        , m_repair_closed_needed(repair_closed)
        , m_repair_pruned_needed(repair_pruned)
        , m_repair_open_needed(repair_open)
        , m_mp_changed(mp_changed)
        , m_new_agent(new_agent)
        , m_lost_agent(lost_agent)
    {}

    std::shared_ptr<const ItagsProblemInputs> DtagsProblemInputsChanges::getNewInputs() const
    {
        return m_new_inputs;
    }

    void DtagsProblemInputsChanges::setNewInputs(std::shared_ptr<const ItagsProblemInputs> new_inputs)
    {
        m_new_inputs = new_inputs;
    }

    bool DtagsProblemInputsChanges::getMotionPlanChanged() const
    {
        return m_mp_changed;
    }

    void DtagsProblemInputsChanges::setMotionPlanChanged(bool mp_change)
    {
        m_mp_changed = mp_change;
    }

    bool DtagsProblemInputsChanges::getNeedUpdateClosed() const
    {
        return m_repair_closed_needed;
    }

    void DtagsProblemInputsChanges::setNeedUpdateClosed(bool repair_closed_needed)
    {
        m_repair_closed_needed = repair_closed_needed;
    }

    bool DtagsProblemInputsChanges::getNeedUpdatePruned() const
    {
        return m_repair_pruned_needed;
    }

    void DtagsProblemInputsChanges::setNeedUpdatePruned(bool repair_pruned_needed)
    {
        m_repair_pruned_needed = repair_pruned_needed;
    }

    bool DtagsProblemInputsChanges::getNeedUpdateOpen() const
    {
        return m_repair_open_needed;
    }

    void DtagsProblemInputsChanges::setNeedUpdateOpen(bool repair_open_needed)
    {
        m_repair_open_needed = repair_open_needed;
    }

    bool DtagsProblemInputsChanges::getLostAgent() const
    {
        return m_lost_agent;
    }

    void DtagsProblemInputsChanges::setLostAgent(bool lost_agent)
    {
        m_lost_agent = lost_agent;
    }

    bool DtagsProblemInputsChanges::getNewAgent() const
    {
        return m_new_agent;
    }

    void DtagsProblemInputsChanges::setNewAgent(bool new_agent)
    {
        m_new_agent = new_agent;
    }
}  // namespace grstapse