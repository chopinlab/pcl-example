#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <original_pcd_file> <update_pcd_file>" << std::endl;
        return -1;
    }

    std::string original_pcd_file = argv[1];
    std::string update_pcd_file = argv[2];

    // 1. 원본 PCD 파일 읽기
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(original_pcd_file, *original_cloud) == -1) {
        PCL_ERROR("Couldn't read the original PCD file \n");
        return -1;
    }
    std::cout << "Loaded original PCD file with " << original_cloud->points.size() << " points." << std::endl;

    // 2. 업데이트할 PCD 파일 읽기
    pcl::PointCloud<pcl::PointXYZ>::Ptr update_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(update_pcd_file, *update_cloud) == -1) {
        PCL_ERROR("Couldn't read the update PCD file \n");
        return -1;
    }
    std::cout << "Loaded update PCD file with " << update_cloud->points.size() << " points." << std::endl;

    // 3. 업데이트할 점 구름의 Convex Hull 계산 (외곽선 찾기)
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(update_cloud);
    convex_hull.setDimension(2);  // x, y 평면 기준으로 2D 외곽선 생성

    std::vector<pcl::Vertices> hull_polygons;  // Convex Hull의 다각형 인덱스들을 저장할 벡터
    convex_hull.reconstruct(*convex_hull_points, hull_polygons);  // Convex Hull 생성

    std::cout << "Computed convex hull with " << convex_hull_points->points.size() << " points." << std::endl;

    // 4. 원본 점 구름에서 Convex Hull 내부에 있는 점들을 제거
    pcl::PointIndices::Ptr points_to_remove(new pcl::PointIndices);

    // CropHull 필터를 사용하여 Convex Hull 내부의 점을 선택
    pcl::CropHull<pcl::PointXYZ> crop_hull_filter;
    crop_hull_filter.setDim(2);  // x, y 평면 기준으로 2D 영역에서 자르기
    crop_hull_filter.setInputCloud(original_cloud);
    crop_hull_filter.setHullCloud(convex_hull_points);
    crop_hull_filter.setHullIndices(hull_polygons);  // Convex Hull 다각형 인덱스 전달
    crop_hull_filter.filter(points_to_remove->indices);

    std::cout << "Found " << points_to_remove->indices.size() << " points inside the convex hull." << std::endl;

    // 원본 점 구름에서 선택된 점을 제거
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(original_cloud);
    extract.setIndices(points_to_remove);
    extract.setNegative(true);  // Convex Hull 내부의 점을 제거
    extract.filter(*original_cloud);
    std::cout << "Removed points inside the convex hull from the original PCD." << std::endl;

    // 5. 업데이트된 점 클라우드를 원본에 추가
    *original_cloud += *update_cloud;

    // 6. 결과를 병합된 PCD 파일로 저장
    std::string merged_pcd_file = "merged_pcd.pcd";
    pcl::io::savePCDFileASCII(merged_pcd_file, *original_cloud);
    std::cout << "Saved merged PCD file with " << original_cloud->points.size() << " points." << std::endl;

    return 0;
}
