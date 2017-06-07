#include <image_transport/simple_publisher_plugin.h>
#include <OpenFace/ResizedImage.h>

class ResizedPublisher : public image_transport::SimplePublisherPlugin<OpenFace::ResizedImage>
{
public:
  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;
};
