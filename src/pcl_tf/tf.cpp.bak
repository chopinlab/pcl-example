#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>

// WGS84 매개변수
const double a = 6378137.0;       // 지구 적도 반경
const double f = 1 / 298.257223563; // 지구 편평율
const double e2 = 2 * f - f * f;  // 제1 이심률의 제곱

// 위도, 경도, 고도를 ECEF 좌표계로 변환
Eigen::Vector3d LatLonHeightToECEF(double lat, double lon, double height) {
    double rad_lat = lat * M_PI / 180.0;
    double rad_lon = lon * M_PI / 180.0;
    
    double N = a / sqrt(1 - e2 * sin(rad_lat) * sin(rad_lat));
    
    double x = (N + height) * cos(rad_lat) * cos(rad_lon);
    double y = (N + height) * cos(rad_lat) * sin(rad_lon);
    double z = (N * (1 - e2) + height) * sin(rad_lat);
    
    return Eigen::Vector3d(x, y, z);
}

// SLAM 로컬 좌표계를 절대 좌표계로 변환
Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local, const Eigen::Vector3d& originECEF, double heading) {
    // heading (방위각)을 라디안으로 변환
    double heading_rad = heading * M_PI / 180.0;

    // 회전 행렬을 정의 (방위각을 기준으로 z 축을 회전)
    Eigen::Matrix3d rotation;
    rotation << cos(heading_rad), -sin(heading_rad), 0,
                sin(heading_rad),  cos(heading_rad), 0,
                0,                 0,                1;

    // 로컬 좌표에 회전 행렬 적용 후, ECEF 좌표계로 변환
    Eigen::Vector3d globalECEF = rotation * local + originECEF;
    return globalECEF;
}

int main(int argc, char** argv) {
    // 명령줄 인수가 적절하게 제공되었는지 확인
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <input_json_file>" << std::endl;
        return -1;
    }

    // 입력 파일 경로
    std::string input_pcd_file = argv[1];
    std::string input_json_file = argv[2];

    // JSON 파일에서 파라미터를 읽음
    nlohmann::json json_data;
    std::ifstream json_file(input_json_file);
    if (!json_file.is_open()) {
        std::cerr << "Unable to open JSON file: " << input_json_file << std::endl;
        return -1;
    }
    json_file >> json_data;
    
    double latitude = json_data["latitude"];
    double longitude = json_data["longitude"];
    double height = json_data["height"];
    double heading = json_data["heading"];

    // 원점의 GPS 좌표를 ECEF 좌표로 변환
    Eigen::Vector3d originECEF = LatLonHeightToECEF(latitude, longitude, height);

    // PCD 파일 읽기
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file: %s\n", input_pcd_file.c_str());
        return -1;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_pcd_file << std::endl;

    // 각 포인트를 절대 좌표계로 변환
    for (auto& point : *cloud) {
        Eigen::Vector3d localPoint(point.x, point.y, point.z);
        Eigen::Vector3d globalPoint = LocalToGlobal(localPoint, originECEF, heading);
        point.x = globalPoint.x();
        point.y = globalPoint.y();
        point.z = globalPoint.z();
    }

    // 변환된 포인트 클라우드를 새로운 PCD 파일로 저장
    std::string output_pcd_file = "output.pcd";
    pcl::io::savePCDFileASCII(output_pcd_file, *cloud);
    std::cout << "Saved " << cloud->points.size() << " data points to " << output_pcd_file << std::endl;

    return 0;
}
