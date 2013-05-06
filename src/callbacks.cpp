#include <stdexcept>
#include <boost/bind.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visp/vpImage.h>

#include "conversion.hh"

#include "callbacks.hh"

void imageCallback(vpImage<unsigned char>& image,
		   const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& info)
{
  try
    {
      rosImageToVisp(image, msg);
    }
  catch(std::exception& e)
    {
      ROS_ERROR_STREAM("dropping frame: " << e.what());
    }
}

void imageCallback(vpImage<unsigned char>& image,
		   std_msgs::Header& header,
		   sensor_msgs::CameraInfoConstPtr& info,
		   const sensor_msgs::Image::ConstPtr& msg,
		   const sensor_msgs::CameraInfoConstPtr& infoConst)
{
  imageCallback(image, msg, info);
  header = msg->header;
  info = infoConst;
}


image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
		  std_msgs::Header& header,
		  sensor_msgs::CameraInfoConstPtr& info)
{
  return boost::bind
    (imageCallback,
     boost::ref(image), boost::ref(header), boost::ref(info), _1, _2);
}


/*
void reconfigureCallback(vpMbEdgeTracker& tracker,
			 vpImage<unsigned char>& I,
			 vpMe& moving_edge,
			 visp_tracker::MovingEdgeConfig& config,
			 uint32_t level)
{
  ROS_INFO("Reconfigure request received.");
  convertMovingEdgeConfigToVpMe(config, moving_edge, tracker);

  //FIXME: not sure if this is needed.
  moving_edge.initMask();

  vpHomogeneousMatrix cMo;
  tracker.getPose(cMo);

  tracker.setMovingEdge(moving_edge);
  tracker.init(I, cMo);

  moving_edge.print();
}*/
