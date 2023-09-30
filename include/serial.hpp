#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <cstring> // for memcpy
#include <sys/time.h>
#include <deque>
#include <mutex>
#define MAX_LEN 64
class SerialData
{
public:
  SerialData(){};
  ~SerialData(){};
  SerialData(const SerialData &sd)
  {
    memcpy(this->storage, sd.storage, sd.len);
    this->len = sd.len;
    this->pack_type = sd.pack_type;
    this->timestamp = sd.timestamp;
  }
  SerialData(uint8_t *data, uint8_t len, uint8_t pack_type, my_time timestamp)
  {
    this->len = len;
    this->pack_type = pack_type;
    this->timestamp = timestamp;
    memcpy(this->storage, data, len);
  }
  uint8_t storage[MAX_LEN];
  uint8_t len;
  uint8_t pack_type;
  // 0-receive_time 1-receive_pose 2-send_flags 3-send_flags&pose
  my_time timestamp;
};

template <typename T>
class ThreadSafeDeque
{

public:
  // 获取容器大小
  int size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_.size();
  }

  // 调整容器大小
  void resize(int size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deque_.resize(size);
  }
  // 获取头部元素
  const T &front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_.front();
  }

  // 判断是否为空
  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_.empty();
  }
  T pop_front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (deque_.empty())
    {
      return T();
    }
    T front = deque_.front();
    deque_.pop_front();
    return front;
  }

  void push_back(const T &item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deque_.push_back(item);
  }

  // 在头部插入元素
  void push_front(const T &item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deque_.push_front(item);
  }

  // 获取尾部元素
  T back()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_.back();
  }

  // 删除尾部元素
  void pop_back()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deque_.pop_back();
  }
  // 返回第二个元素
  const T &second()
  {

    std::lock_guard<std::mutex> lock(mutex_);

    // 如果容器少于两个元素,返回默认元素
    if (deque_.size() < 2)
    {
      return deque_.front();
    }

    // 保留第一个元素
    T front1 = deque_.front();
    deque_.pop_front();

    // 获取第二个元素
    T front2 = deque_.front();

    // 恢复原顺序
    deque_.push_front(front1);

    return front2;
  }
  void updata_size()
  {

    while (this->size() > 9)
    {
      this->pop_back();
    }
  }

private:
  std::mutex mutex_;
  std::deque<T> deque_;
};

#endif