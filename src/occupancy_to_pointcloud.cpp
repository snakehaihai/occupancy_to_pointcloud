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
        grid_sub_ = nh_.subscribe("/map", 1, &GridToPointCloud::gridCb, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
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
                    for (double z = 0.0; z <= desired_height; z += desired_height / num_layers)
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
    const double desired_height = 1.0; // change this to the height you want
    const unsigned int num_layers = 100; // change this to increase or decrease the point density in the z-axis
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_to_pointcloud_node");
    GridToPointCloud g2p;
    ros::spin();
    return 0;
}

