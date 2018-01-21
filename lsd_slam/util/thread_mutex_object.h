/*
 * ThreadMutexObject.h
 *
 *  Created on: 11 May 2012
 *      Author: thomas
 */

#ifndef THREADMUTEXOBJECT_H_
#define THREADMUTEXOBJECT_H_
#include <thread>
#include <mutex>
#include <chrono>


template <class T>
class ThreadMutexObject
{
public:
  ThreadMutexObject()
  {}

  ThreadMutexObject(T initialValue)
    : object(initialValue),
    lastCopy(initialValue)
  {}

  void assignValue(T newValue)
  {
    std::unique_lock<std::mutex> lock(mutex);

    object = lastCopy = newValue;

    lock.unlock();
  }

  std::mutex & getMutex()
  {
    return mutex;
  }

  T & getReference()
  {
    return object;
  }

  void assignAndNotifyAll(T newValue)
  {
    std::unique_lock<std::mutex> lock(mutex);

    object = newValue;

    signal.notify_all();

    lock.unlock();
  }

  void notifyAll()
  {
    std::unique_lock<std::mutex> lock(mutex);

    signal.notify_all();

    lock.unlock();
  }

  T getValue()
  {
    std::unique_lock<std::mutex> lock(mutex);

    lastCopy = object;

    lock.unlock();

    return lastCopy;
  }

  T waitForSignal()
  {
    std::unique_lock<std::mutex> lock(mutex);

    signal.wait(mutex);

    lastCopy = object;

    lock.unlock();

    return lastCopy;
  }

  T getValueWait(int wait = 33000)
  {
    std::this_thread::sleep(std::chrono::milliseconds(wait));

    std::unique_lock<std::mutex> lock(mutex);

    lastCopy = object;

    lock.unlock();

    return lastCopy;
  }

  T & getReferenceWait(int wait = 33000)
  {
    std::this_thread::sleep(std::chrono::milliseconds(wait));

    std::unique_lock<std::mutex> lock(mutex);

    lastCopy = object;

    lock.unlock();

    return lastCopy;
  }

  void operator++(int)
  {
    std::unique_lock<std::mutex> lock(mutex);

    object++;

    lock.unlock();
  }

private:
  T object;
  T lastCopy;
  std::mutex mutex;
  std::condition_variable_any signal;
};

#endif /* THREADMUTEXOBJECT_H_ */
