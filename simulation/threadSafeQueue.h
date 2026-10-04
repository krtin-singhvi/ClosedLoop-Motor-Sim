#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue
{
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable condition;
    bool closed = false;

public:

    bool push(const T& value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (closed)
                return false;

            queue.push(value);
        }

        condition.notify_one();
        return true;
    }

    bool pop(T& value)
    {
        std::unique_lock<std::mutex> lock(mutex);

        condition.wait(lock, [this] {
            return !queue.empty() || closed;
        });

        if (queue.empty())
            return false;

        value = queue.front();
        queue.pop();

        return true;
    }

    void close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            closed = true;
        }

        condition.notify_all();
    }
};
