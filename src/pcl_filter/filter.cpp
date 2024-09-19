#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    std::string input_pcd_file = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file \n");
        return -1;
    }

    std::cout << "Loaded PCD file with " << cloud->points.size() << " points." << std::endl;

    // 사용자가 입력한 좌표 4개 (x, y) -> z는 임의로 0으로 고정
    pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // polygon_cloud->points.push_back(pcl::PointXYZ(1.0, 1.0, 0.0));  // 첫 번째 좌표
    // polygon_cloud->points.push_back(pcl::PointXYZ(5.0, 1.0, 0.0));  // 두 번째 좌표
    // polygon_cloud->points.push_back(pcl::PointXYZ(5.0, 5.0, 0.0));  // 세 번째 좌표
    // polygon_cloud->points.push_back(pcl::PointXYZ(1.0, 5.0, 0.0));  // 네 번째 좌표

    polygon_cloud->points.push_back(pcl::PointXYZ(-0.5, -0.5, 0.0));  // 첫 번째 좌표
    polygon_cloud->points.push_back(pcl::PointXYZ(-0.5, 0.5, 0.0));  // 두 번째 좌표
    polygon_cloud->points.push_back(pcl::PointXYZ(2.0, -0.5, 0.0));  // 세 번째 좌표
    polygon_cloud->points.push_back(pcl::PointXYZ(2.0, 0.5, 0.0));  // 네 번째 좌표

    // ConvexHull 생성 (사용자가 입력한 좌표로 다각형 영역을 만들기 위해)
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(polygon_cloud);
    convex_hull.setDimension(2);  // 2D 다각형으로 설정

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hull_polygons;  // 다각형 인덱스 저장

    convex_hull.reconstruct(*hull_cloud, hull_polygons);  // Convex Hull을 생성하고 다각형 인덱스를 저장

    // CropHull 필터로 다각형 내부의 포인트들만 남김
    pcl::CropHull<pcl::PointXYZ> crop_hull_filter;
    crop_hull_filter.setDim(2);  // 2D 다각형으로 필터링
    crop_hull_filter.setInputCloud(cloud);
    crop_hull_filter.setHullCloud(hull_cloud);
    crop_hull_filter.setHullIndices(hull_polygons);  // Convex Hull 다각형 인덱스 전달

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_hull_filter.filter(*cropped_cloud);  // 다각형 내부의 포인트만 남김

    std::cout << "Cropped cloud contains " << cropped_cloud->points.size() << " points." << std::endl;

    // 결과를 새로운 PCD 파일로 저장
    std::string output_pcd_file = "trab_filter_after.pcd";
    pcl::io::savePCDFileASCII(output_pcd_file, *cropped_cloud);
    std::cout << "Saved cropped PCD to " << output_pcd_file << std::endl;

    return 0;
}