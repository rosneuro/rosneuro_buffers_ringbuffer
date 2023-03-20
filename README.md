# ROS-Neuro RingBuffer package
The package provides the implementation of a RingBuffer. The implemented data replacement policy is FIFO.

## Example of RingBuffer (with YAML configuration)
```cpp
#include <ros/ros.h>
#include "rosneuro_buffers/RingBuffer.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "ringbuffer");
  
  rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>();
  if(buffer->configure("RingBufferCfgTest") == false) {
  	ROS_ERROR("RingBuffer configuration failed");
	return false;
  }
  ROS_INFO("RingBuffer configuration succeeded");
  // ...
  // ...
  delete buffer;
  return 0;
}
```
```
RingBufferCfgTest:
  name: ringbuffer
  type: RingBufferFloat
  params: 
    size: 20
```


## RingBuffer templates
The RingBuffer provides plugin for three type of Eigen data: *int*, *float*, and *double*. The different plugin can be loaded by providing the following types in the yaml configuration:
- RingBufferInt (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)
- RingBufferFloat (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)
- RingBufferDouble (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)

