#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// num indicates the max number of elements to read, -1 means read till the end
template <class T>
std::vector<T> read_bag(std::string file_name, std::string topic,
                        unsigned int num) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<T> msgs;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (m.instantiate<T>() != NULL) {
      msgs.push_back(*m.instantiate<T>());
      if (msgs.size() > num) break;
    }
  }
  bag.close();
  if (msgs.empty())
    ROS_WARN("Fail to find '%s' in '%s', make sure md5sum are equivalent.",
             topic.c_str(), file_name.c_str());
  else
    ROS_INFO("Get data!");
  return msgs;
}

template <typename T>
void write_bag(std::string file_name, std::string topic_name,
               const std::vector<T>& msgs) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  for (const auto& it : msgs) bag.write(topic_name, it.header.stamp, it);
  ROS_INFO("Write %zu messages as %s in file %s", msgs.size(),
           topic_name.c_str(), file_name.c_str());
  bag.close();
}

