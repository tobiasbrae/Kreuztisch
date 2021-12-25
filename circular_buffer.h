/*
*	Author: Tobias Braechter
*
*	file: circular_buffer.h
*	Date: 21.11.2017
*	
*	Implements a circular buffer for any datatype
*
*/

#include <stdint.h>
#include <stdlib.h>

#ifndef CIRCULAR_BUFFER_H
	#define CIRCULAR_BUFFER_H
	
	#define MAX_BUFFER_SIZE 10000
	
	typedef struct
	{
		void *buffer;
		void *cache;
		uint8_t isNew;
		uint16_t bufferSize;
		uint16_t read;
		uint16_t write;
	}buffer;
	
	buffer *buffer_createBuffer(uint8_t typeSize, uint16_t bufferSize);
	#define createBuffer(typeSize, bufferSize) buffer_createBuffer(sizeof(typeSize), bufferSize)

	uint16_t buffer_getNextIndex(buffer *this, uint8_t typeSize, uint16_t start, uint16_t offset);
	
	void buffer_putItem(buffer *this, uint8_t typeSize, char *value);
	#define putItem(this, type, value) 	{																	\
											type buffercache = value; 										\
											type* bufferpointer = &buffercache;								\
											buffer_putItem(this, sizeof(type), (char*) bufferpointer);		\
										}
	
	uint16_t buffer_getFreeSpace(buffer *this, uint8_t typeSize);
	uint16_t buffer_getUsedSpace(buffer *this, uint8_t typeSize);
	#define getFreeSpace(this) buffer_getFreeSpace(this, 1)
	#define getFreeItemSpace(this, type) buffer_getFreeSpace(this, sizeof(type))
	#define getUsedSpace(this) buffer_getUsedSpace(this, 1)
	#define getUsedItemSpace(this, type) buffer_getUsedSpace(this, sizeof(type))
	
	uint8_t buffer_hasSpaceForItem(buffer *this, uint8_t typeSize);
	#define hasSpaceForItem(this, typeSize) buffer_hasSpaceForItem(this, sizeof(typeSize))

	#define hasNextItem(this, type) buffer_hasNextItems(this, sizeof(type), 1)
	#define hasNextItems(this, type, amount) buffer_hasNextItems(this, sizeof(type), amount)
	uint8_t buffer_hasNextItems(buffer *this, uint8_t typeSize, uint16_t amount);
	
	char *buffer_getNextItem(buffer *this, uint8_t typeSize, uint16_t offset);
	#define getItem(this, type) *(type*) buffer_getNextItem(this, sizeof(type), 0)
	#define getNextItem(this, type, offset) *(type*) buffer_getNextItem(this, sizeof(type), offset)

	void buffer_deleteItem(buffer *this, uint8_t typeSize);
	#define deleteItem(this, type) buffer_deleteItem(this, sizeof(type))

	void buffer_deleteItems(buffer *this, uint8_t typeSize, uint16_t amount);
	#define deleteItems(this, type, amount) buffer_deleteItems(this, sizeof(type), amount)
	
	void resetBuffer(buffer *this);
#endif
