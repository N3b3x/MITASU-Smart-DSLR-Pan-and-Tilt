#include "../inc/CommandBuffer.hpp"

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Puts a command in the command buffer
 * 
 *  @param[in] command Character array containing the command
 * 
 *  @return 0 if buffer is full
 *          1 if command saved successfully, 
 *          2 if the passed pointer is a NULL pointer
 * ----------------------------------------------------------------------------------------------------------------------------------
*/

uint8_t CommandBuffer::putCommand(char* command)
{
    // If its a NULL pointer, return 2
    if(command == NULL)
        return 2;

    // If buffer is full, return 0
    if(_full)
        return 0;   

    // Otherwise, iterate through command extracting out characters and saving in buffer
    uint8_t i = 0;
    while((command[i] != '\0') && (i < (MAX_COMMAND_LENGTH-1))){
        buffer[_tail][i++] = command[i];    // Save command character then increase the iterator
    }
    buffer[_tail][i] = '\0';                // Finish the character array with a '\0', in order to be NULL terminated

    _tail = (_tail+1) % BUFFERSIZE;         // Increment the tail by 1, use modulo to wrap around if _tail passes BUFFERSIZE
    
    if (_tail == _head)                     // if the tail pointer, after being incremented, is equal to head, the buffer is full
        _full = 1;

    _numCommandsStored++;                   // Increment the number of commands stored
    return 1;
}


/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Gets and removes the next command received from the buffer. FIFO setup.
 * 
 *  @param[in] command Character array that the extracted command will be stored in
 * 
 *  @return 0 if buffer is empty
 *          1 if getting command was successful, 
 *          2 if the passed pointer is a NULL pointer
 * ----------------------------------------------------------------------------------------------------------------------------------
*/

uint8_t CommandBuffer::getCommand(char* command)
{
    // If its a NULL pointer, return 2
    if(command == NULL)
        return 2;

    // If the buffer is empty, NULL terminate character array's first position then return 0 
    if(isEmpty()){
        command[0] = '\0';
        return 0;
    }

    // Otherwise, go to head pointer and extract the command stored at that index
    uint8_t i = 0;
    while ((buffer[_head][i] != '\0') && (i < MAX_COMMAND_LENGTH-1)){
        command[i] = buffer[_head][i++];    // Save character from buffer then increase interator
    }
    command[i] = '\0';                      // Finish the character array with a '\0', in order to be NULL terminated

    _full = 0;                              // Since we just created more space, set _full to 0 (Much quicker than checking if it's set then clearing it)
    _head = (_head+1) % BUFFERSIZE;         // Increment the head by 1, use modulo to wrap around if _head passes BUFFERSIZE

    _numCommandsStored--;                   // Decrement the number of command stored since we just extracted a command
    return 1;
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Peeks the next command received from the buffer without removing it. FIFO setup.
 * 
 *  @param[in] command  Character array that the extracted command will be stored in
 * 
 *  @return 0 if buffer is empty
 *          1 if peeking was successful, 
 *          2 if the passed pointer is a NULL pointer
 * ----------------------------------------------------------------------------------------------------------------------------------
*/

uint8_t CommandBuffer::peekCommand(char* command)
{
    // If its a NULL pointer, return 2
    if(command == NULL)
        return 2;

    // If the buffer is empty, NULL terminate character array and return 0 
    if(isEmpty()){
        command[0] = '\0';
        return 0;
    }

    // Go to head pointer and extract the command stored at that index
    uint8_t i = 0;
    while ((buffer[_head][i] != '\0') && (i < MAX_COMMAND_LENGTH-1))
    {
        command[i] = buffer[_head][i++];    // Save character in buffer then increase interator
    }
    command[i] = '\0';                      // Finish the character array with a '\0', in order to be NULL terminated

    return 1;
}