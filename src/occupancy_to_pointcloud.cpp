#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
class GridToPointCloud
{
public:
    GridToPointCloud()
    {
        // Subscribe and Advertise topics
        grid_sub_ = nh_.subscribe("/map", 1, &GridToPointCloud::gridCb, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);

        // Get parameters from the parameter server (or set defaults if not found)
        nh_.param("starting_height", starting_height_, -1.0);
        nh_.param("ending_height", ending_height_, 1.0);
        nh_.param("height_step", height_step_, 0.1);
    }

private:
    void gridCb(const nav_msgs::OccupancyGridConstPtr& grid)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        double resolution = grid->info.resolution;
        double origin_x = grid->info.origin.position.x;
        double origin_y = grid->info.origin.position.y;

        for (unsigned int i = 0; i < grid->info.width; ++i)
        {
            for (unsigned int j = 0; j < grid->info.height; ++j)
            {
                if (grid->data[j * grid->info.width + i] > 0) // if occupied
                {
                    for (double z = starting_height_; z <= ending_height_; z += height_step_)
                    {
                        pcl::PointXYZ point;
                        point.x = i * resolution + origin_x;
                        point.y = j * resolution + origin_y;
                        point.z = z;
                        cloud.push_back(point);
                    }
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_out;
        pcl::toROSMsg(cloud, cloud_out);
        cloud_out.header = grid->header;
        cloud_out.header.frame_id = "map"; // or your preferred frame
        cloud_pub_.publish(cloud_out);
    }

    ros::NodeHandle nh_;
    ros::Subscriber grid_sub_;
    ros::Publisher cloud_pub_;
    double starting_height_;
    double ending_height_;
    double height_step_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_to_pointcloud_node");
    GridToPointCloud g2p;
    ros::spin();
    return 0;
}


