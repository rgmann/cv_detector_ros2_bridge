// Copyright [2025] [Robert Vaughan]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __MESSAGE_QUEUE_H__
#define __MESSAGE_QUEUE_H__

#include <chrono>
#include <queue>
#include <condition_variable>
#include <mutex>


template <class T>
class sync_queue {
public:

    /**
     * Construct thread-safe queue with optional maximum size
     *
     * @param max_size Maximum number of elements that may in the queue (0 for unlimited)
     */
    sync_queue(size_t max_size = 0) : max_size_(max_size) {};

    /**
     * Add a single element to the end of the queue and notify all blocked consumers.
     * If the queue has a maximum size, the queue first checks that adding the
     * new element would not queue the queue to exceed the maximum size.
     * 
     * @param message Message to add to the queue
     * @return True if the message is added to the queue; false otherwise
     */
    bool push(T message)
    {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        if ((max_size_ == 0) || (queue_.size() < max_size_))
        {
            queue_.push(message);
            lock.unlock();
            cv_.notify_all();
            return true;
        }
        return false;
    }

    /**
     * Remove and return the message at the front of queue. If there are no
     * messages in the queue, block until a message is available or until
     * the timeout period elapses.
     * 
     * @param message Message from beginning of the queue
     * @param timeout Timeout period in milliseconds
     * @return True if a message is received within the timeout; false otherwise
     */
    bool pop(T& message, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(cv_mutex_);

        bool received_message = false;

        if (queue_.size() == 0)
        {
            cv_.wait_for(lock, timeout, [this] {
                return queue_.size() > 0;
            });
        }

        if ((received_message = (queue_.size() > 0)))
        {
            message = queue_.front();
            queue_.pop();
        }

        return received_message;
    }


private:

    size_t max_size_;

    std::queue<T> queue_;

    std::condition_variable cv_;

    std::mutex cv_mutex_;

};


#endif // __MESSAGE_QUEUE_H__