/*
 * eventdispatch.h
 *
 *  Created on: Dec 19, 2025
 *      Author: philt
 */

#ifndef ASYNC_EVENT_DISPATCH_H_
#define ASYNC_EVENT_DISPATCH_H_

#include <coroutine>

void register_awaiter(void* key, std::coroutine_handle<> h);
void resume_awaiter(void* key);

#endif /* ASYNC_EVENT_DISPATCH_H_ */
