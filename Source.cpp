// This is a queue manager, you have fixed size buffer(2048 bytes) and you want to manage a bunch of queues.
// The element of each queue is byte. 
// Use the buffer as effective as possible, support queue as more as possible, run as fast as possible.

// Interfaces: 

// Queue* create_queue()
// void enqueue_byte(Queue *i_q, byte i_value)
// byte dequeue_byte(Queue *i_q) 
// void destroy_queue(Queue *i_q)


#include <stdint.h>																			   

#include <assert.h>
#include <stdio.h>
#include <memory.h>
#include <time.h>

// features: create, destroy, enqueue and dequeue will be all O(1) with amortized analysis.	Enqueue may become slower as available memory decrease.

// The more queue you use (The max number in the history. For example: queue number changed from 3->10->2->6, the max number is 10), 
// the less bytes you can push into your queue, empty queue have 0 byte.
// The relation is:  QueueCount * 4 + Bytes <= BufferSize(2048).

typedef unsigned char byte;
typedef unsigned int uint;

// a node is 32 bits, 11 bits for queue pos(max 2048), 11 for queue len(max 2048), 
// 9 bits for next node(max 511 queues, a value 0 was used to test null), and last 1 bit to mark this node is used or not
typedef unsigned int Node;
typedef Node Queue;

#pragma region VariablesSetting
const uint bufferSize = 2048;       // test case is 2048 bytes, but you can change to a bigger one

const uint nextPosNull = 0;
unsigned char data[bufferSize]; // our buffer

uint spacer;   // used to handle possible invalid operation for buffer.

uint lastNodePos = 0; // latest node pos, init 0, temporary bigest pos
uint nodeCount = 0;
int totalQueueSize = bufferSize;
int totalFreeSize = bufferSize;

uint intervalNodeCount = 0;
#pragma endregion

#pragma region Node Operation
inline uint GetQueuePos(Node *i_value)
{
	return (*i_value & 0xffe00000) >> 21;	// get first 11 bytes
}
inline void SetQueuePos(Node *io_node, uint i_value)
{
	*io_node = (*io_node & 0x001fffff) + (i_value << 21);
}

inline uint GetQueueLen(Node *i_value)
{
	return (*i_value & 0x001ffc00) >> 10;	// get middle 11 bytes
}
inline void SetQueueLen(Node *io_node, uint i_value)
{
	*io_node = (*io_node & 0xffe003ff) + (i_value << 10);
}

inline uint GetNextNodePos(Node *i_value)
{
	return (*i_value & 0x000003fe) >> 1;	// get last 10 bytes
}
inline void SetNextNodePos(Node *io_node, uint i_value)
{
	*io_node = (*io_node & 0xfffffc01) + (i_value << 1);
}

// 1 means being used, 0 means free to use
inline bool GetNodeUsed(Node *i_value)
{
	return (*i_value & 0x1) == 1;
}
inline void SetNodeUsed(Node *io_node, bool i_used)
{
	*io_node = (*io_node) / 2 * 2 + (i_used ? 1 : 0);
}

inline void SetNodeData(Node *io_node, uint i_queuePos, uint i_queueLen, uint i_nextNodePos, bool i_used = true)
{
	*io_node = (i_queuePos << 21) + (i_queueLen << 10) + (i_nextNodePos << 1) + (i_used ? 1 : 0);
}

inline Node* GetNodePtrFromNodePos(uint i_nodePos)
{
	return reinterpret_cast<Node *>(&data[bufferSize - i_nodePos * 4]);
}
inline uint GetNodePosFromNodePtr(Node *i_node)
{
	return static_cast<uint>((reinterpret_cast<long long>(&data[bufferSize]) - reinterpret_cast<long long>(i_node)) / 4);
}
#pragma endregion

// push all queue forward and let each queue has the same interval.
void OrganizeAllQueue()
{
	if (totalFreeSize < 0)
	{
		printf_s("On out of memory!\n");
		assert(false);
	}

	// when an re-organization is required, it means at least one queue is exist
	uint firstNodePos = 1;
	while (!GetNodeUsed(GetNodePtrFromNodePos(firstNodePos)))
	{
		++firstNodePos;
	}

	uint tempNodePos = firstNodePos;
	uint tempTotalLen = 0;

	// normally we don't need a reverse placement, but it may be required.
	bool reversePlaceRequired = false;

	for (uint i = 0; i < nodeCount; ++i)
	{
		Node *tempNode = GetNodePtrFromNodePos(tempNodePos);
		uint tempPos = GetQueuePos(tempNode);
		uint targetPos = tempTotalLen + i * totalFreeSize / nodeCount;
		if (tempPos != targetPos)
		{
			// most of the time, all queue will move to one direction, the targetPos should always smaller than tempPos, which prevent data lose
			// but for few times, some queues have to move forward, in this case, set reversePlacement to true
			if (targetPos > tempPos)
			{
				reversePlaceRequired = true;
			}
			else
			{
				memcpy(&data[targetPos], &data[tempPos], GetQueueLen(tempNode) * sizeof(byte));
				SetQueuePos(tempNode, targetPos);
			}
		}
		tempTotalLen += GetQueueLen(tempNode);
		tempNodePos = GetNextNodePos(tempNode);
	}

	tempNodePos = lastNodePos;
	if (reversePlaceRequired)
	{
		for (uint i = 0; i < nodeCount; ++i)
		{
			while (!GetNodeUsed(GetNodePtrFromNodePos(tempNodePos)))
			{
				--tempNodePos;
			}				      // will always be able to find an used previous node
			Node *tempNode = GetNodePtrFromNodePos(tempNodePos);
			uint tempPos = GetQueuePos(tempNode);
			tempTotalLen -= GetQueueLen(tempNode);
			uint targetPos = tempTotalLen + (nodeCount - 1 - i) * totalFreeSize / nodeCount;
			if (tempPos != targetPos)
			{
				// handle move forward cases
				if (targetPos > tempPos)
				{
					memcpy(&data[targetPos], &data[tempPos], GetQueueLen(tempNode) * sizeof(byte));
					SetQueuePos(tempNode, targetPos);
				}
			}
			--tempNodePos;
		}
	}
}

// another special function to push next queue can be implemented here for enqueue, solve most of the problem
// you should make sure a node is behind this node when use this function
void PushNextQueue(Node* i_node)
{
	Node* nextNode = GetNodePtrFromNodePos(GetNextNodePos(i_node));				// since there is a node behind this node, nextPos must exist
	uint nextNextPos = GetNextNodePos(nextNode);

	uint nextQueuePos = GetQueuePos(nextNode);
	uint nextQueueLen = GetQueueLen(nextNode);
	uint nextNextQueuePos;
	if (nextNextPos != nextPosNull)			// a next next node exist
	{
		Node *nextNextNode = GetNodePtrFromNodePos(nextNextPos);
		nextNextQueuePos = GetQueuePos(nextNextNode);					// we get nextNext queue pos
	}
	else
	{
		nextNextQueuePos = totalQueueSize;					// we get the total size
	}

	if (nextQueuePos + nextQueueLen < nextNextQueuePos)		  // at least 1 space between next node and nextNext node
	{
		uint targetPos = nextQueuePos + (nextNextQueuePos - nextQueuePos - nextQueueLen + 1) / 2;     // push (interval + 1) / 2 byte 
		memcpy(&data[targetPos], &data[nextQueuePos], nextQueueLen * sizeof(byte));
		SetQueuePos(nextNode, targetPos);
	}
	else							  //if can not push(no interval between nextNode and nextNext Node), organize all queues
	{
		OrganizeAllQueue();
	}
}


#pragma	region Interfaces
Queue* create_queue()
{
	Node* resultNode;
	++nodeCount;

	if (intervalNodeCount == 0)
	{
		totalQueueSize -= 4;
		totalFreeSize -= 4;
		if (lastNodePos == 0)
		{
			resultNode = GetNodePtrFromNodePos(lastNodePos + 1);
			SetNodeData(resultNode, 0, 0, nextPosNull);
			++lastNodePos;
			return resultNode;
		}
		else
		{
			Node *previousNode = GetNodePtrFromNodePos(lastNodePos);

			resultNode = GetNodePtrFromNodePos(lastNodePos + 1);
			SetNextNodePos(previousNode, lastNodePos + 1);
			++lastNodePos;

			if (static_cast<int>(GetQueuePos(previousNode) + GetQueueLen(previousNode)) <= totalQueueSize)
			{
				SetNodeData(resultNode, (GetQueuePos(previousNode) + GetQueueLen(previousNode) + totalQueueSize) / 2, 0, nextPosNull);
				return resultNode;
			}
			else
			{
				SetNodeData(resultNode, (GetQueuePos(previousNode) + GetQueueLen(previousNode) + totalQueueSize) / 2, 0, nextPosNull);
				OrganizeAllQueue();
				return resultNode;
			}
		}
	}
	else
	{
		--intervalNodeCount;
		uint previousPos = 0;
		while (GetNodeUsed(GetNodePtrFromNodePos(previousPos + 1)))
		{
			++previousPos;
		}				// will always be able to find an unused interval

		resultNode = GetNodePtrFromNodePos(previousPos + 1);
		if (previousPos == 0)
		{
			uint nextPos = previousPos + 2;
			while (nextPos <= lastNodePos && !GetNodeUsed(GetNodePtrFromNodePos(nextPos)))					// only search when no previous node
			{
				++nextPos;
			}
			if (nextPos != lastNodePos + 1)
				SetNodeData(resultNode, 0, 0, nextPos);
			else										   // if we can not find the next used node
				SetNodeData(resultNode, 0, 0, nextPosNull);

			return resultNode;
		}
		else
		{
			Node *previousNode = GetNodePtrFromNodePos(previousPos);
			uint nextPos = GetNextNodePos(previousNode);		// will never be an invalid value
			SetNextNodePos(previousNode, previousPos + 1);
			uint tempQueuePos = (GetQueuePos(previousNode) + GetQueueLen(previousNode) + GetQueuePos(GetNodePtrFromNodePos(nextPos))) / 2;	  // get the middle point to set a new queue
			SetNodeData(resultNode, tempQueuePos, 0, nextPos);
			return resultNode;
		}
	}
}

void enqueue_byte(Queue *i_q, byte i_value)
{
	uint nextNodePos = GetNextNodePos(i_q);
	SetQueueLen(i_q, GetQueueLen(i_q) + 1);
	--totalFreeSize;
	if (nextNodePos == nextPosNull)
	{
		if (static_cast<int>(GetQueuePos(i_q) + GetQueueLen(i_q)) <= totalQueueSize)
		{
			data[GetQueuePos(i_q) + GetQueueLen(i_q) - 1] = i_value;
		}
		else
		{
			OrganizeAllQueue();			// there is no next Node available, organize all queues
			if (static_cast<int>(GetQueuePos(i_q) + GetQueueLen(i_q)) <= totalQueueSize)
			{
				data[GetQueuePos(i_q) + GetQueueLen(i_q) - 1] = i_value;
			}
			else
			{
				printf_s("On out of memory!\n");
				assert(false);
			}
		}
	}
	else
	{
		Node* nextNode = GetNodePtrFromNodePos(nextNodePos);		 // in this case, nextNode will always exist
		if (GetQueuePos(i_q) + GetQueueLen(i_q) <= GetQueuePos(nextNode))
		{
			data[GetQueuePos(i_q) + GetQueueLen(i_q) - 1] = i_value;
		}
		else
		{
			PushNextQueue(i_q);				// next node available, try to push it
			if (GetQueuePos(i_q) + GetQueueLen(i_q) <= GetQueuePos(nextNode))
			{
				data[GetQueuePos(i_q) + GetQueueLen(i_q) - 1] = i_value;
			}
			else
			{
				printf_s("On out of memory!\n");
				assert(false);
			}
		}
	}
}

byte dequeue_byte(Queue *i_q)
{
	byte result;
	if (GetQueueLen(i_q) == 0)
	{
		printf_s("Illegal Operation!\n");
		result = 0;
	}
	else
	{
		++totalFreeSize;
		result = data[GetQueuePos(i_q)];
		SetQueuePos(i_q, GetQueuePos(i_q) + 1);
		SetQueueLen(i_q, GetQueueLen(i_q) - 1);
	}
	return result;
}

void destroy_queue(Queue *i_q)
{
	++intervalNodeCount;
	--nodeCount;
	totalFreeSize += GetQueueLen(i_q);
	SetNodeUsed(i_q, false);

	uint tempNodePos = GetNodePosFromNodePtr(i_q);
	uint previousNodePos = tempNodePos - 1;
	while (previousNodePos > 0 && !GetNodeUsed(GetNodePtrFromNodePos(previousNodePos)))
	{
		--previousNodePos;
	}
	if (previousNodePos != 0)			// after search forward, if a previous node exist
	{
		SetNextNodePos(GetNodePtrFromNodePos(previousNodePos), GetNextNodePos(i_q));
	}
}

void reset_buffer()
{
	lastNodePos = 0;
	nodeCount = 0;
	totalQueueSize = bufferSize;
	totalFreeSize = bufferSize;
	intervalNodeCount = 0;
}
#pragma endregion

#include <vector>
#include <algorithm>

#pragma region TestCases
void Test1()
{
	// normal test
	reset_buffer();
	Queue * q0 = create_queue();
	enqueue_byte(q0, 0);
	enqueue_byte(q0, 1);
	Queue * q1 = create_queue();
	enqueue_byte(q1, 3);
	enqueue_byte(q0, 2);
	enqueue_byte(q1, 4);
	printf("%d ", dequeue_byte(q0));
	printf("%d\n", dequeue_byte(q0));
	enqueue_byte(q0, 5);
	enqueue_byte(q1, 6);
	printf("%d ", dequeue_byte(q0));
	printf("%d\n", dequeue_byte(q0));
	destroy_queue(q0);
	printf("%d ", dequeue_byte(q1));
	printf("%d ", dequeue_byte(q1));
	printf("%d\n", dequeue_byte(q1));
	destroy_queue(q1);

	printf_s("\nPast test 1\n\n");
}

void Test2()
{
	// create test, 
	reset_buffer();
	std::vector<Queue *> testQueues;
	testQueues.reserve(513);
	int numberOfQueue = 511;
	// we support at most 511 queues
	// if you want to create 512 queues, there will be a wrong access operation on our buffer
	// if you want to create 513 queues, memory is not enough and an assert will work

	for (int i = 0; i < numberOfQueue; ++i)
	{
		testQueues.push_back(create_queue());
	}
	std::random_shuffle(testQueues.begin(), testQueues.end());
	for (int i = numberOfQueue - 1; i >= 0; --i)
	{
		destroy_queue(testQueues[i]);
	}
	printf_s("\nPast test 2\n\n");
}

void Test3()
{
	// separate create test
	reset_buffer();

	int firstAllocateQueue = 10, firstFreeQueue = 5, secondAllocateQueue = 8, usedQueue;
	usedQueue = firstAllocateQueue;
	std::vector<std::pair<Queue *, int>> queues;

	// create some queues
	for (int i = 0; i < firstAllocateQueue; ++i)
	{
		queues.push_back(std::make_pair(create_queue(), 0));
	}
	// enqueue until full
	for (unsigned int i = 0; i < bufferSize - usedQueue * 4; ++i)
	{
		int tempRandom = rand() % firstAllocateQueue;
		enqueue_byte(queues[tempRandom].first, i);
		++queues[tempRandom].second;
	}

	// destroy some queues
	for (int i = 0; i < firstFreeQueue; ++i)
	{
		destroy_queue(queues.back().first);
		queues.pop_back();
	}
	usedQueue -= firstFreeQueue;

	// create some queues again
	for (int i = 0; i < secondAllocateQueue; ++i)
	{
		queues.push_back(std::make_pair(create_queue(), 0));
	}
	usedQueue += secondAllocateQueue;

	// enqueue until full again
	for (int i = 0; i < totalFreeSize; ++i)
	{
		int tempRandom = rand() % firstAllocateQueue;
		enqueue_byte(queues[tempRandom].first, i);
		++queues[tempRandom].second;
	}

	// dequeue all data
	for (unsigned int i = 0; i < queues.size(); ++i)
	{
		for (int j = 0; j < queues[i].second; ++j)
		{
			printf_s("%d, ", dequeue_byte(queues[i].first));
		}
	}

	// destroy all queues
	for (unsigned int i = 0; i < queues.size(); ++i)
	{
		destroy_queue(queues[i].first);
	}

	printf_s("\nPast test 3\n\n");
}
#pragma endregion

int main()
{
	srand(static_cast<unsigned int>(time(0)));

	Test1();
	Test2();
	Test3();

	printf_s("Past all tests!\n");
	return 0;
}
