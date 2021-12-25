/*
*	Author: Tobias Braechter
*
*	file: circular_buffer.c
*	Date: 21.11.2017
*	
*	Implements a circular buffer for all data-Types
*
*/

#include "circular_buffer.h"

buffer *buffer_createBuffer(uint8_t typeSize, uint16_t bufferSize)
{
	if(typeSize * bufferSize > MAX_BUFFER_SIZE)
		bufferSize = MAX_BUFFER_SIZE / typeSize;
	buffer *buf = malloc(sizeof(buffer));
	buf->buffer = malloc(typeSize * bufferSize);
	buf->cache = malloc(typeSize);
	buf->isNew = 1;
	buf->bufferSize = bufferSize * typeSize;
	buf->read = 0;
	buf->write = 0;
	return buf;
}

uint16_t buffer_getNextIndex(buffer *this, uint8_t typeSize, uint16_t start, uint16_t offset)
{
	if(this)
		return (start + typeSize * offset) % this->bufferSize;
	return 0;
}

void buffer_putItem(buffer *this, uint8_t typeSize, char *value)
{
	if(this)
	{
		if(buffer_hasSpaceForItem(this, typeSize))
		{
			this->isNew = 0;
			char *pointer = (char*) this->buffer;
			for(int i = 0; i < typeSize; i++)
				pointer[buffer_getNextIndex(this, sizeof(char), this->write, i)] = value[i];
			this->write = buffer_getNextIndex(this, sizeof(char), this->write, typeSize);
		}
	}
}

uint16_t buffer_getUsedSpace(buffer *this, uint8_t typeSize)
{
	if(this)
	{
		if(this->write > this->read)
			return (this->write - this->read) / typeSize;
		else if(this->write < this->read)
			return (this->bufferSize - this->read + this->write) / typeSize;
		else if(this->isNew)
			return 0;
		else
			return this->bufferSize / typeSize;
	}
	return 0;
}

uint16_t buffer_getFreeSpace(buffer *this, uint8_t typeSize)
{
	if(this)
	{
		return (this->bufferSize - getUsedSpace(this)) / typeSize;
	}
	return 0;
}

uint8_t buffer_hasSpaceForItem(buffer *this, uint8_t typeSize)
{
	if(this)
		if(getFreeSpace(this) >= typeSize)
			return 1;
	return 0;
}

uint8_t buffer_hasNextItems(buffer *this, uint8_t typeSize, uint16_t amount)
{
	if(this)
		if(getUsedSpace(this) >= typeSize * amount)
			return 1;
	return 0;
}

char *buffer_getNextItem(buffer *this, uint8_t typeSize, uint16_t offset)
{
	if(this)
	{
		char *pointer = (char*) this->buffer;
		free(this->cache);
		this->cache = malloc(typeSize);
		char *cache = (char*) this->cache;
		for(int i = 0; i < typeSize; i++)
			if(getUsedSpace(this) >= typeSize)
				*(cache + i) = *(pointer + buffer_getNextIndex(this, sizeof(char), this->read, offset * typeSize + i));
			else
				*(cache + i) = 0;
		return this->cache;
	}
	return 0;
}

void buffer_deleteItem(buffer *this, uint8_t typeSize)
{
	if(this)
	{
		if(getUsedSpace(this) >= typeSize)
		{
			char *pointer = (char*) this->buffer;
			for(int i = 0; i < typeSize; i++)
				pointer[buffer_getNextIndex(this, sizeof(char), this->read, i)] = 0;
			this->read = buffer_getNextIndex(this, typeSize, this->read, 1);
			if(this->read == this->write)
				this->isNew = 1;
		}
	}
}

void buffer_deleteItems(buffer *this, uint8_t typeSize, uint16_t amount)
{
	if(this)
	{
		for(uint16_t i = 0; i < amount; i++)
			buffer_deleteItem(this, typeSize);
	}
}

void resetBuffer(buffer *this)
{
	if(this)
	{
		this->read = 0;
		this->write = 0;
		this->isNew = 1;
	}
}
