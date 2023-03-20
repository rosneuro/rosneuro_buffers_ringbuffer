# ROS-Neuro RingBuffer package
The package provides the implementation of a RingBuffer. The implemented data replacement policy is FIFO.

## Usage
The buffer can be configured by means of the set function (*bool Buffer\<T\>::set(unsigned int nrows, unsigned int ncols)*) or by yaml configuration and parameter server. The only mandatory parameter is the **size** of the buffer. In the case the buffer is used without being configured, it will throws a std::runtime exception.

For usability, the internal resources of the buffer are set during the first call to the function *bool RingBuffer\<T\>::add(const DynamicMatrix\<T\>& in)*. This effectively reduces the performances during the first call, but it allows to deduce the number of channels at runtime.

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

