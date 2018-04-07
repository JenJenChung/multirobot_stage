# include <math.h>
# include <nav_msgs/OccupancyGrid.h>
# include <geometry_msgs/Pose.h>
# include <Eigen/Dense>
//#, OpenCV?, 

Eigen::MatrixXd map2polar(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose robot, int n_th, double thresh=50 ){
    Eigen::MatrixXd coff(n_th,3);
    double res = map.info.resolution;
    int i = 0;
    for (double theta = 0; theta<2*M_PI; theta = theta + 2*M_PI/n_th ){
        int l = 0;
        bool looking = true;
        coff(i,0) = theta;
        while (looking){
            double x = round(sin(theta)*l + robot.position.x/res);
            double y = round(cos(theta)*l + robot.position.y/res);
            if (x > map.info.width || y > map.info.height || x < 0 || y < 0){
                looking = false;
            } else {
                if (map.data[x,y]>thresh && coff(i,1)==0){
                    coff(i,1) = l*res;
                }
                if (map.data[x,y]==-1){
                    coff(i,2) = l*res;
                }
                l = l++;
            }
        }
        i++;
    }
    return coff;
}

int main(int argc, char **argv){
    std::string file = argv[1];
    Eigen::Vector2d robot;
    std::cout << "Reading from " << file << std::endl << ". Robot position: " << robot << std::endl;

    return 0;
}