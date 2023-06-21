#ifndef FSM_HPP_
#define FSM_HPP_

#include <inttypes.h>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"

class FiniteStateMachine {
    public:

    enum FSM_Event : uint8_t{
        EVT_NONE = 0,
        EVT_RECEIVED_STX = 1,
        EVT_RECEIVED_ETX = 2,
        EVT_RECEIVED_ESC = 3,
        EVT_RECEIVED_DATA = 4,

        _EVT_GUARD_LEN = 5,
        _EVT_INVALID = 6
    };

    enum FSM_State : uint8_t{
        STATE_IDLE = 0,
        STATE_FRAME_STARTED = 1,
        STATE_BYTE_ESCAPED = 2,

        _STATE_GUARD_LEN = 3,
        _STATE_INVALID_ = 4
    };

    const char* GetStateName(FSM_State state) const {
        if(state >= _STATE_GUARD_LEN)
        {
            return "ERROR";
        }

        const char* pData[] = {
            "STATE_IDLE",
            "STATE_FRAME_STARTED",
            "STATE_BYTE_ESCAPED"
        };
        
        return pData[state];
    }

    const char* GetEventName(FSM_Event ev) const {
        if(ev >= _EVT_GUARD_LEN)
        {
            return "ERROR";
        }

        const char* pData[] = {
            "EVT_NONE",
            "EVT_RECEIVED_STX",
            "EVT_RECEIVED_ETX",
            "EVT_RECEIVED_ESC",
            "EVT_RECEIVED_DATA"
        };

        return pData[ev];
    }
    
    using TransitionFn = std::function<void(uint8_t)>;

    FiniteStateMachine(FSM_State initial_state, TransitionFn on_reset, rclcpp::Node& parent);


    void AddTransition(
        FSM_State current_state,
        FSM_Event event,
        FSM_State next_state,
        TransitionFn callback
    );

    // Returns true if transition is defined,
    // False on reset
    bool ProcessEvent(FSM_Event ev, uint8_t input);

    private:

    struct TransitionDescriptor_
    {
        FSM_State next_state;
        TransitionFn callback;
        bool is_active = false;

        TransitionDescriptor_()
        {
            this->next_state = _STATE_INVALID_;
            this->callback = nullptr;
            this->is_active = false;
        }

        TransitionDescriptor_(FSM_State next_state_, TransitionFn callback_)
        {
            this->next_state = next_state_;
            this->callback = callback_;
            this->is_active = true;
        }
    };

    TransitionFn on_reset_;
    FSM_State initial_state_;
    FSM_State current_state_;
    FSM_Event last_event_;

    rclcpp::Node& parent_;

    void Reset_();
    
    TransitionDescriptor_ transition_table_[_STATE_GUARD_LEN][_EVT_GUARD_LEN];
};

#endif