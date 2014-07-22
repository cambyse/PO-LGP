#ifndef POINTCLOUDSUBSCRIBER_H
#define POINTCLOUDSUBSCRIBER_H

#include <string>

class PointCloudSubscriber
{
private:
    struct sPointCloudSubscriber *s;
public:
    PointCloudSubscriber(const std::string& topic_name);
};

#endif // POINTCLOUDSUBSCRIBER_H
