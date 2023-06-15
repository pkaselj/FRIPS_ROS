#include "rover_interface/fsm.hpp"

FiniteStateMachine::FiniteStateMachine(FSM_State initial_state, TransitionFn on_reset, rclcpp::Node& parent)
:   on_reset_(on_reset), initial_state_(initial_state), parent_(parent)
{
    RCLCPP_DEBUG(parent_.get_logger(), " [FSM] Initialized FMS to state %s", GetStateName(initial_state));

    current_state_ = initial_state;
    last_event_ = EVT_NONE;

    // Allocate N_STATES*N_EVENTS table
    for (size_t i_state = 0; i_state < _STATE_GUARD_LEN; i_state++)
    {
        for (size_t j_evt = 0; j_evt < _EVT_GUARD_LEN; j_evt++)
        {
            transition_table_[i_state][j_evt].next_state = _STATE_INVALID_;
            transition_table_[i_state][j_evt].callback = nullptr;
            transition_table_[i_state][j_evt].is_active = false;
        }
    }
    
}

static inline bool is_valid_event_(FiniteStateMachine::FSM_Event ev)
{
    return ev < FiniteStateMachine::_EVT_GUARD_LEN;
}

static inline bool is_valid_state_(FiniteStateMachine::FSM_State st)
{
    return st < FiniteStateMachine::_STATE_GUARD_LEN;
}

void FiniteStateMachine::AddTransition(FSM_State current_state, FSM_Event event, FSM_State next_state, TransitionFn callback)
{
    RCLCPP_DEBUG(
            parent_.get_logger(),
            " [FSM] Adding Transition: State [%s] + Event [%s] = State [%s]",
            GetStateName(current_state),
            GetEventName(event),
            GetStateName(next_state)
    );

    if (!is_valid_state_(current_state) || !is_valid_event_(event) || !is_valid_state_(next_state))
    {
        RCLCPP_WARN(
            parent_.get_logger(),
            " [FSM] Cannot Add Transition: State [%s] + Event [%s] = State [%s]",
            GetStateName(current_state),
            GetEventName(event),
            GetStateName(next_state)
        );

        return;
    }

    transition_table_[current_state][event] = TransitionDescriptor_(next_state, callback);

    RCLCPP_DEBUG(
            parent_.get_logger(),
            " [FSM] Added Transition: State [%s] + Event [%s] = State [%s]",
            GetStateName(current_state),
            GetEventName(event),
            GetStateName(next_state)
    );
}

bool FiniteStateMachine::ProcessEvent(FSM_Event ev, uint8_t input)
{
    RCLCPP_DEBUG(
            parent_.get_logger(),
            " [FSM] Processing event [%s] in current state [%s]",
            GetEventName(ev),
            GetStateName(current_state_)
    );

    if(ev == EVT_NONE)
    {
        last_event_ = ev;
        // Do nothing
        return true;
    }

    const auto& transition = transition_table_[current_state_][ev];

    if (!transition.is_active)
    {
        RCLCPP_WARN(
                parent_.get_logger(),
                " [FSM] Transition not defined (State [%s], Event [%s])",
                GetStateName(current_state_),
                GetEventName(ev)
        );

        Reset_();
        return false;
    }
    
    if(!transition.callback)
    {
        RCLCPP_WARN(
            parent_.get_logger(),
            " [FSM] Callback not callable (State [%s], Event [%s])",
            GetStateName(current_state_),
            GetEventName(ev)
        );
        Reset_();
        return false;
    }

    transition.callback(input);
    
    auto previous_state = current_state_;

    current_state_ = transition.next_state;
    last_event_ = ev;

    RCLCPP_DEBUG(
        parent_.get_logger(),
        " [FSM] Transition for (State [%s], Event [%s]) processed. New state [%s]",
        GetStateName(previous_state),
        GetEventName(ev),
        GetStateName(current_state_)
    );

    return true;
}

void FiniteStateMachine::Reset_()
{
    RCLCPP_WARN(parent_.get_logger(), " [FSM] Reset");

    current_state_ = initial_state_;
    last_event_ = EVT_NONE;

    if (on_reset_)
    {
        on_reset_;
    }
    
}
