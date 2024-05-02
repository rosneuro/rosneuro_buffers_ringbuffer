#include <gtest/gtest.h>
#include <RingBuffer.hpp>

namespace rosneuro {
    class BufferRingTestSuite : public ::testing::Test {
        public:
            BufferRingTestSuite() {};
            ~BufferRingTestSuite() {};
            virtual void SetUp(void) {
                ring_buffer = new RingBuffer<double>();
            }
            virtual void TearDown(void) {
                delete ring_buffer;
            }
            RingBuffer<double>* ring_buffer;
        };

    template<typename T>
    DynamicMatrix<T> generate_frame(int nrows, int ncols, int index) {
        rosneuro::DynamicMatrix<T> frame = rosneuro::DynamicMatrix<T>::Zero(nrows, ncols);
        for (auto i = 0; i<frame.rows(); i++)
            frame.row(i) = (index*nrows + i + 1)*rosneuro::DynamicMatrix<T>::Constant(nrows, ncols, 1);
        return frame;
    }

    TEST_F(BufferRingTestSuite, ConfigureTest) {
        EXPECT_FALSE(ring_buffer->configure());

        ring_buffer->params_["size"] = 100;
        EXPECT_TRUE(ring_buffer->configure());
    }

    TEST_F(BufferRingTestSuite, AddTest) {
        DynamicMatrix<double> test_data(3, 4);

        ring_buffer->resize(3, 4);
        ring_buffer->params_["size"] = 10;
        ring_buffer->configure();
        ring_buffer->is_set_ = true;

        EXPECT_TRUE(ring_buffer->add(test_data));
    }

    TEST_F(BufferRingTestSuite, AddMismatchedColumnsTest) {
        DynamicMatrix<double> initial_data(2, 4);
        DynamicMatrix<double> mismatched_data(3, 5);

        ring_buffer->params_["size"] = 10;
        ring_buffer->configure();

        EXPECT_TRUE(ring_buffer->add(initial_data));
        EXPECT_TRUE(ring_buffer->add(mismatched_data));
        EXPECT_EQ(ring_buffer->cols(), mismatched_data.cols());
    }

    TEST_F(BufferRingTestSuite, Integration) {
        constexpr unsigned int frame_rows = 3;
        constexpr unsigned int frame_cols = 10;

        ring_buffer->params_["size"] = 20;
        ASSERT_TRUE(ring_buffer->configure());

        int niter = 0;
        DynamicMatrix<double> frame, output;

        try {
            do {
                frame = generate_frame<double>(frame_rows, frame_cols, niter);
                ring_buffer->add(frame);
                niter++;
            } while (!ring_buffer->isfull());
        } catch (std::runtime_error& e) {
            FAIL();
        }
        SUCCEED();
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_ring_buffer");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}