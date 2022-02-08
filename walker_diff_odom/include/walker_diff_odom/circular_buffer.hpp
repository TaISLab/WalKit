
#ifndef CIRCULAR_BUFFER_HPP_
#define CIRCULAR_BUFFER_HPP_

#include <vector>

class CircularBuffer{
  public:
      CircularBuffer(int size=10) : stored_vals_(size, 0.0), next_pos_(0), sum_(0.0), is_full_(false), getLastCounter(0){
  }

      void add(int val);

      int getAverage();

      int getLast();

      void clear();

      int getCount();

  private:
    std::vector<int> stored_vals_;
    long unsigned int next_pos_;
    int sum_;
    bool is_full_;
    int getLastCounter;
};

#endif // CIRCULAR_BUFFER_HPP_