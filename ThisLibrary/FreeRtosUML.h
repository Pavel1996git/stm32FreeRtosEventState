/*
 * FreeRtosUML.h
 *
 *  Created on: Apr 8, 2024
 *      Author: pavel
 */

#ifndef FREERTOSUML_H_
#define FREERTOSUML_H_


#include "cmsis_os.h"
#include "queue.h"
#include "stdint.h"
#include "stdio.h"
//#include <sys/_stdint.h>
#define DELAY_QUEUE 15000

#define NUM_EVENTS 10
#define NUM_STATES 10

/**
 * The initializeTransitionEvent function initializes the transition table between states based on events.
 *
 * @param table Two-dimensional array representing the transition table.
 * @param num_states The number of states in the state machine.
 * @param num_events The number of possible events.
 */
void initializeTransitionEvent(int8_t table[][NUM_EVENTS], uint8_t num_states, uint8_t num_events);
/**
 * The function initializeTransitionFork initializes the transition table for state forking.
 *
 * @param table A two-dimensional array representing the transition table.
 * @param num_states The number of states in the finite state machine.
 * @param num_new_states The number of new states that can be reached from each current state.
 */
void initializeTransitionFork(int8_t table[][NUM_STATES], uint8_t num_states, uint8_t num_new_states);
/**
 * The initializeTransitionEndState function initializes the end state table.
 *
 * @param table The array representing the end state table.
 * @param num_states The number of states in the finite state machine.
 */
void initializeTransitionEndState(int8_t table[], uint8_t num_states);

/**
 * The addToTransitionFork function adds a transition from the initial state to the new state in the fork state table.
 *
 * @param initial_state The initial state of the transition.
 * @param new_state The new state to transition to.
 */
void addToTransitionFork(int8_t initial_state, int8_t  new_state);
/**
 * The addToTransitionEvent function adds a transition from the initial state to the new state triggered by the specified event.
 *
 * @param initial_state The initial state of the transition.
 * @param new_state The new state to transition to.
 * @param event The event triggering the transition.
 */
void addToTransitionEvent(int8_t initial_state, int8_t  new_state, int8_t event);
/**
 * The addToTransitionEndState function adds an end state for the specified initial state.
 *
 * @param initial_state The initial state to add the end state for.
 * @param new_state The new end state to add.
 */
void addToTransitionEndState(int8_t  initial_state, int8_t  new_state);
/**
 * The handleTransition function determines the new state based on the current state and the event received.
 *
 * @param event The event triggering the state transition.
 * @param currentState The current state.
 * @return The new state after the transition. Returns -1 if there is no transition defined for the given event and current state.
 */
void createStateQueueMapping(int8_t state, osMessageQueueId_t queueHandle);
/**
 * The getQueueForState function retrieves the message queue associated with a given state.
 *
 * @param state The state for which to retrieve the associated message queue.
 * @return The handle of the message queue associated with the specified state.
 */
void MessageQueueEvent(int8_t  state, int8_t  event);
/**
 * The sendToTransitionFork function sends messages to the message queues associated with the new states based on the fork transitions from the current state.
 *
 * @param currentState The current state from which fork transitions are examined.
 */
void sendToTransitionFork(int8_t  currentState);
/**
 * The sendToTransitionEvent function sends messages to the message queue associated with the new state determined by the transition event from the current state.
 *
 * @param currentState The current state from which the transition event originates.
 * @param event The event triggering the state transition.
 */
void sendToTransitionEvent(int8_t  currentState, int8_t  event);
/**
 * The sendToTransitionEndState function sends a message to the message queue associated with the end state of the transition from the current state.
 *
 * @param currentState The current state from which the transition ends.
 */
void sendToTransitionEndState(int8_t  currentState);
/**
 * The waitForOwnState function waits for the current state to be received from the message queue associated with it.
 *
 * @param currentState Pointer to the variable holding the current state.
 */
void waitForOwnState(int8_t * currentState);
/**
 * The waitForOwnEvent function waits for an event associated with the current state by receiving it from the corresponding message queue.
 *
 * @param currentState The current state for which the event is awaited.
 * @param event Pointer to the variable where the received event will be stored.
 */
void waitForOwnEvent(int8_t  currentState, int8_t * event);

extern int8_t transitionTable[NUM_STATES][NUM_EVENTS];
extern int8_t transitionEndState[NUM_STATES];
extern int8_t transitionForkState[NUM_STATES][NUM_STATES];
extern osMessageQueueId_t stateQueueMappings[NUM_STATES];

#endif /* FREERTOSUML_H_ */
