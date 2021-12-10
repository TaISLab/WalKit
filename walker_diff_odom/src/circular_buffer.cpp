

#include "walker_diff_odom/circular_buffer.hpp"


void CircularBuffer::add(int val) {
        // remove old val
        sum_ -= stored_vals_[next_pos_];
        // add new val
        sum_ += val;
        // store new val
        stored_vals_[next_pos_] = val;
        // update pointer
        next_pos_++;
        // is buffer full?
        is_full_ = ( is_full_ | ( next_pos_ >= stored_vals_.size() ) );
        next_pos_ = ( next_pos_ % stored_vals_.size() ) ;
}

int CircularBuffer::getAverage(){
        int data_size;
        int ans;
        if (is_full_){
            data_size =  stored_vals_.size();
        } else {
            data_size =  next_pos_;
        }

        if (data_size){
            ans = sum_ / data_size;
        } else {
            ans = 0;
        }

        return ans;
}

int CircularBuffer::getLast(){ 
        int ans;
        if (next_pos_>0){
          ans = stored_vals_[next_pos_-1];
        } else{
          if (is_full_){
            ans = stored_vals_[stored_vals_.size()-1];
          } else {
            ans = 0;
          }
        }

        return ans;
}

void CircularBuffer::clear(){ 
    next_pos_ = 0;
    is_full_ = false;
    sum_ = 0;
    stored_vals_ = std::vector<int>(stored_vals_.size(), 0.0);
}