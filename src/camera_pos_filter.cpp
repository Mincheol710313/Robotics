#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>

class CameraPosFilter
{
public:
    CameraPosFilter()
    {
        // Initialize ROS node and subscribers/publishers
        ros::NodeHandle nh_;
        camera_pos_sub_ = nh_.subscribe("camera_pos", 10, &CameraPosFilter::cameraPosCallback, this);
        filter_camera_pos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("filter_camera_pos", 1);

        // Initialize the queue size to store recent 10 values
        queue_size_ = 10;
    }

    void cameraPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        // Add the received values (x, y, theta) to their respective queues
        if (msg->x == 0 && msg->y == 0){
            geometry_msgs::Pose2D filter_msg;
            filter_msg.x = 0;
            filter_msg.y = 0;
            filter_msg.theta = 0;
            filter_camera_pos_pub_.publish(filter_msg);
            }
        else {
            camera_pos_queue_[0].push(msg->x);
            camera_pos_queue_[1].push(msg->y);
            camera_pos_queue_[2].push(msg->theta);
            }

        // Remove the oldest value from each queue if the size exceeds 10
        for (int i = 0; i < 3; ++i)
        {
            if (camera_pos_queue_[i].size() > queue_size_)
            {
                camera_pos_queue_[i].pop();
            }
        }

        // Calculate the averages of the last 10 values for x, y, and theta
        double sums[3] = {0};
        for (int i = 0; i < 3; ++i)
        {
            std::queue<double> temp_queue = camera_pos_queue_[i];
            while (!temp_queue.empty())
            {
                sums[i] += temp_queue.front();
                temp_queue.pop();
            }
        }
        double averages[3] = {0};
        for (int i = 0; i < 3; ++i)
        {
            averages[i] = sums[i] / camera_pos_queue_[i].size();
        }

        // Check if the averages are greater than 5, then publish 0 for x, y, and theta
        if (abs(averages[0] - msg->x) >= 5 || abs(averages[1] - msg->y) >= 5)
        {
            geometry_msgs::Pose2D filter_msg;
            filter_msg.x = 0;
            filter_msg.y = 0;
            filter_msg.theta = 0;
            filter_camera_pos_pub_.publish(filter_msg);
        }
        else
        {
            filter_camera_pos_pub_.publish(*msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_pos_sub_;
    ros::Publisher filter_camera_pos_pub_;
    std::queue<double> camera_pos_queue_[3]; // One queue for each field: x, y, theta
    int queue_size_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_pos_filter");
    CameraPosFilter filter;
    ros::spin();
    return 0;
}
