#include <image_transport/simple_subscriber_plugin.h>
#include <OpenFace/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<OpenFace::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename OpenFace::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};
