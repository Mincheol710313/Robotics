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
        camera_pos_sub_ = nh_.subscribe("camera_pos", 5, &CameraPosFilter::cameraPosCallback, this);
        filter_camera_pos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("filter_camera_pos", 1);

        // Initialize the queue size to store recent 10 values
        queue_size_ = 5;
    }

    void cameraPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        // Add the received values (x, y, theta) to their respective queues
        if (msg->x == 0 && msg->y == 0)
        {
            geometry_msgs::Pose2D filter_msg;
            filter_msg.x = 0;
            filter_msg.y = 0;
            filter_msg.theta = 0;
            filter_camera_pos_pub_.publish(filter_msg);
        }
        else
        {
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

        // Calculate the variances of the last 10 values for x, y, and theta
        double sums[3] = {0};
        double squares[3] = {0};
        for (int i = 0; i < 3; ++i)
        {
            std::queue<double> temp_queue = camera_pos_queue_[i];
            while (!temp_queue.empty())
            {
                double value = temp_queue.front();
                sums[i] += value;
                squares[i] += value * value;
                temp_queue.pop();
            }
        }
        double variances[3] = {0};
        for (int i = 0; i < 3; ++i)
        {
            int queue_size = camera_pos_queue_[i].size();
            if (queue_size > 1) // Avoid division by zero
            {
                double mean = sums[i] / queue_size;
                variances[i] = (squares[i] - (sums[i] * sums[i]) / queue_size) / (queue_size - 1);
            }
            else
            {
                variances[i] = 0;
            }
        }

        // Check if the variances are greater than a threshold, then publish 0 for x, y, and theta
        double variance_threshold = 0.35; // Set your desired threshold here
        std::cout << variances[0] << ", " << variances[1] << std::endl;
        if (variances[0] >= variance_threshold || variances[1] >= variance_threshold)
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
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
