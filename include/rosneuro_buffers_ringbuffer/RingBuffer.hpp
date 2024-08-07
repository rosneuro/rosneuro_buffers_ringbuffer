#ifndef ROSNEURO_RINGBUFFER_HPP
#define ROSNEURO_RINGBUFFER_HPP

#include <rosneuro_buffers/Buffer.hpp>
#include <gtest/gtest_prod.h>

namespace rosneuro {
    template<typename T>
    class RingBuffer : public Buffer<T> {
        public:
            RingBuffer(void);
            ~RingBuffer(void) {};
            bool configure(void);
            bool add(const DynamicMatrix<T>& in);

        protected:
            unsigned int size_;

        private:
            FRIEND_TEST(BufferRingTestSuite, ConfigureTest);
            FRIEND_TEST(BufferRingTestSuite, AddTest);
            FRIEND_TEST(BufferRingTestSuite, AddMismatchedColumnsTest);
            FRIEND_TEST(BufferRingTestSuite, Integration);
    };

    template<typename T>
    RingBuffer<T>::RingBuffer(void) {}

    template<typename T>
    bool RingBuffer<T>::configure(void) {
        if (!Buffer<T>::getParam(std::string("size"), this->size_)) {
            ROS_ERROR("[Buffer] Cannot find param size");
            return false;
        }
        return true;
    }

    template<typename T>
    bool RingBuffer<T>::add(const DynamicMatrix<T>& in) {
        if(!this->is_set_) {
            this->is_set_ = this->set(this->size_, in.cols());
            ROS_WARN("[%s] First apply: the buffer is set ([%d %ld])", this->name().c_str(), this->rows(), in.cols());
        }

        if(this->cols() != in.cols() ) {
            this->is_set_ = this->set(this->rows(), in.cols());
            ROS_WARN("[%s] Buffer and incoming data has different number of columns ([%d %d] vs. [%ld %ld])."
                     "The buffer will be automatically resized.", this->name().c_str(), this->rows(),
                      this->cols(), in.rows(), in.cols());
        }
        DynamicMatrix<T> c_buffer = this->data_.bottomRows(this->data_.rows() - in.rows());
        this->data_ << c_buffer, in;

        return true;
    }
}

#endif
