/*
 * FreeRtosUML.c
 *
 *  Created on: Apr 8, 2024
 *      Author: pavel
 */


#include "FreeRtosUML.h"


// An array representing the transition table, where each element corresponds to a transition between states triggered by events.
int8_t transitionTable[NUM_STATES][NUM_EVENTS];

// An array representing the end states after transitions, where each element corresponds to the final state reached after transitioning from an initial state.
int8_t transitionEndState[NUM_STATES];

// An array representing the possible forked states, where each element corresponds to the states that can be reached simultaneously from an initial state.
int8_t transitionForkState[NUM_STATES][NUM_STATES];

// An array mapping each state to its corresponding message queue ID, allowing communication between tasks based on their current states.
osMessageQueueId_t stateQueueMappings[NUM_STATES];

/**
 * The initializeTransitionEvent function initializes the transition table between states based on events.
 *
 * @param table Two-dimensional array representing the transition table.
 * @param num_states The number of states in the state machine.
 * @param num_events The number of possible events.
 */
void initializeTransitionEvent(int8_t table[][NUM_EVENTS], uint8_t num_states, uint8_t num_events)
{
    // Filling the transition table with -1 values, indicating no transitions.
    for (uint8_t i = 0; i < num_states; ++i) {
        for (uint8_t j = 0; j < num_events; ++j) {
            table[i][j] = -1; // Setting the value -1 for each element of the transition table.
        }
    }
}

/**
 * The function initializeTransitionFork initializes the transition table for state forking.
 *
 * @param table A two-dimensional array representing the transition table.
 * @param num_states The number of states in the finite state machine.
 * @param num_new_states The number of new states that can be reached from each current state.
 */
void initializeTransitionFork(int8_t table[][NUM_STATES], uint8_t num_states, uint8_t num_new_states)
{
    // Fills the state table with -1 values, indicating no transitions.
    for (uint8_t i = 0; i < num_states; ++i) {
        for (uint8_t j = 0; j < num_new_states; ++j) {
            table[i][j] = -1;
        }
    }
}


/**
 * The initializeTransitionEndState function initializes the end state table.
 *
 * @param table The array representing the end state table.
 * @param num_states The number of states in the finite state machine.
 */
void initializeTransitionEndState(int8_t table[], uint8_t num_states)
{
    // Fills the end state table with -1 values, indicating no transitions.
    for (uint8_t i = 0; i < num_states; ++i) {
        table[i] = -1;
    }
}

/**
 * The addToTransitionFork function adds a transition from the initial state to the new state in the fork state table.
 *
 * @param initial_state The initial state of the transition.
 * @param new_state The new state to transition to.
 */
void addToTransitionFork(int8_t initial_state, int8_t new_state)
{
    // Check if the provided states are valid
    if (initial_state >= NUM_STATES || new_state >= NUM_STATES) {
        // Output an error message or take other measures
        return;
    }

    // Set the transition in the transition fork state table
    transitionForkState[initial_state][new_state] = 1; // Here, 1 can be any other value depending on your logic
}

/**
 * The addToTransitionEvent function adds a transition from the initial state to the new state triggered by the specified event.
 *
 * @param initial_state The initial state of the transition.
 * @param new_state The new state to transition to.
 * @param event The event triggering the transition.
 */
void addToTransitionEvent(int8_t initial_state, int8_t new_state, int8_t event)
{
    // Check if the provided states and event are valid
    if (initial_state >= NUM_STATES || new_state >= NUM_STATES || event >= NUM_EVENTS) {
        // Output an error message or take other measures
        return;
    }

    // Fill the transition table
    transitionTable[initial_state][event] = new_state;
}


/**
 * The addToTransitionEndState function adds an end state for the specified initial state.
 *
 * @param initial_state The initial state to add the end state for.
 * @param new_state The new end state to add.
 */
void addToTransitionEndState(int8_t initial_state, int8_t new_state)
{
    // Check if the provided states are valid
    if (initial_state >= NUM_STATES || new_state >= NUM_STATES) {
        // Output an error message or take other measures
        return;
    }

    // Fill the transition end state table
    transitionEndState[initial_state] = new_state;
}


/**
 * The handleTransition function determines the new state based on the current state and the event received.
 *
 * @param event The event triggering the state transition.
 * @param currentState The current state.
 * @return The new state after the transition. Returns -1 if there is no transition defined for the given event and current state.
 */
int8_t handleTransition(int8_t event, int8_t currentState)
{
    // Check if there is a new state for the given event
    if (transitionTable[currentState][event] != -1) {
        // Retrieve the new state
        int8_t newState = transitionTable[currentState][event];
        return newState;
    } else {
        // No transition defined for the given event and current state
        return -1;
    }
}


/**
 * The createStateQueueMapping function creates an association between a state and a message queue.
 *
 * @param state The state to associate with the message queue.
 * @param queueHandle The handle of the message queue.
 */
void createStateQueueMapping(int8_t state, osMessageQueueId_t queueHandle)
{
    // Create the association between the state and the message queue
    stateQueueMappings[state] = queueHandle;
}


/**
 * The getQueueForState function retrieves the message queue associated with a given state.
 *
 * @param state The state for which to retrieve the associated message queue.
 * @return The handle of the message queue associated with the specified state.
 */
osMessageQueueId_t getQueueForState(int8_t state)
{
    // Retrieve the message queue associated with the state
    return stateQueueMappings[state];
}


/**
 * The MessageQueueState function sends a state value to the message queue associated with the specified state.
 *
 * @param state The state value to send to the message queue.
 */
void MessageQueueState(int8_t state)
{
    // Send the state value to the message queue associated with the specified state
    xQueueSend(getQueueForState(state), &state, pdMS_TO_TICKS(DELAY_QUEUE));
}


/**
 * The MessageQueueEvent function sends an event value to the message queue associated with the specified state.
 *
 * @param state The state associated with the message queue.
 * @param event The event value to send to the message queue.
 */
void MessageQueueEvent(int8_t state, int8_t event)
{
    // Send the event value to the message queue associated with the specified state
    xQueueSend(getQueueForState(state), &event, pdMS_TO_TICKS(DELAY_QUEUE));
}


/**
 * The sendToTransitionFork function sends messages to the message queues associated with the new states based on the fork transitions from the current state.
 *
 * @param currentState The current state from which fork transitions are examined.
 */
void sendToTransitionFork(int8_t currentState)
{
    // Check if the current state is valid
    if (currentState >= NUM_STATES) {
        // Output an error message or take other actions
        return;
    }

    // Iterate over all possible new states
    for (int8_t new_state = 0; new_state < NUM_STATES; ++new_state) {
        // Check if there is a transition from the current state to the new state
        if (transitionForkState[currentState][new_state] == 1) {
            // Send a message to the message queue associated with the corresponding new state
            MessageQueueState(new_state);
        }
    }
}


/**
 * The sendToTransitionEvent function sends messages to the message queue associated with the new state determined by the transition event from the current state.
 *
 * @param currentState The current state from which the transition event originates.
 * @param event The event triggering the state transition.
 */
void sendToTransitionEvent(int8_t currentState, int8_t event)
{
    // Determine the new state based on the transition event and current state
    int8_t newState = handleTransition(event, currentState);
    // Send a message to the message queue associated with the new state
    MessageQueueState(newState);
}


/**
 * The sendToTransitionEndState function sends a message to the message queue associated with the end state of the transition from the current state.
 *
 * @param currentState The current state from which the transition ends.
 */
void sendToTransitionEndState(int8_t currentState)
{
    // Retrieve the end state of the transition from the transitionEndState array
    int8_t endState = transitionEndState[currentState];
    // Send a message to the message queue associated with the end state
    MessageQueueState(endState);
}


/**
 * The waitForOwnState function waits for the current state to be received from the message queue associated with it.
 *
 * @param currentState Pointer to the variable holding the current state.
 */
void waitForOwnState(int8_t *currentState)
{
    // Wait to receive the current state from the associated message queue
    xQueueReceive(getQueueForState(*currentState), currentState, portMAX_DELAY);
}



/**
 * The waitForOwnEvent function waits for an event associated with the current state by receiving it from the corresponding message queue.
 *
 * @param currentState The current state for which the event is awaited.
 * @param event Pointer to the variable where the received event will be stored.
 */
void waitForOwnEvent(int8_t currentState, int8_t *event)
{
    // Wait for receiving the event associated with the current state from the corresponding message queue
    xQueueReceive(getQueueForState(currentState), event, portMAX_DELAY);
}


