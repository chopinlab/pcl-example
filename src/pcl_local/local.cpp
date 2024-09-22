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

// 절대 좌표계를 로컬 좌표계로 변환 (map1의 로컬 기준으로 변환)
Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global, const Eigen::Vector3d& originECEF, double heading) {
    // heading (방위각)을 라디안으로 변환
    double heading_rad = heading * M_PI / 180.0;

    // 회전 행렬의 역행렬을 정의
    Eigen::Matrix3d rotation;
    rotation << cos(heading_rad), sin(heading_rad), 0,
               -sin(heading_rad), cos(heading_rad), 0,
                0,                0,               1;

    // 글로벌 좌표를 로컬 좌표로 변환
    Eigen::Vector3d local = rotation * (global - originECEF);
    return local;
}

int main(int argc, char** argv) {
    // 명령줄 인수가 적절하게 제공되었는지 확인
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <map1.pcd> <map1.json> <map2.pcd> <map2.json>" << std::endl;
        return -1;
    }

    // 명령줄 인수로 받은 파일 경로
    std::string map1_pcd_file = argv[1];
    std::string map1_json_file = argv[2];
    std::string map2_pcd_file = argv[3];
    std::string map2_json_file = argv[4];

    // JSON 파일에서 map1과 map2의 파라미터를 읽음
    nlohmann::json map1_json, map2_json;
    
    std::ifstream map1_file(map1_json_file);
    if (!map1_file.is_open()) {
        std::cerr << "Unable to open map1 JSON file: " << map1_json_file << std::endl;
        return -1;
    }
    map1_file >> map1_json;

    std::ifstream map2_file(map2_json_file);
    if (!map2_file.is_open()) {
        std::cerr << "Unable to open map2 JSON file: " << map2_json_file << std::endl;
        return -1;
    }
    map2_file >> map2_json;

    // map1과 map2의 GPS 및 heading 정보 가져오기
    double lat1 = map1_json["latitude"];
    double lon1 = map1_json["longitude"];
    double height1 = map1_json["height"];
    double heading1 = map1_json["heading"];

    double lat2 = map2_json["latitude"];
    double lon2 = map2_json["longitude"];
    double height2 = map2_json["height"];
    double heading2 = map2_json["heading"];

    // map1 원점의 GPS 좌표를 ECEF 좌표로 변환
    Eigen::Vector3d originECEF1 = LatLonHeightToECEF(lat1, lon1, height1);
    
    // map2 원점의 GPS 좌표를 ECEF 좌표로 변환
    Eigen::Vector3d originECEF2 = LatLonHeightToECEF(lat2, lon2, height2);

    // map2의 PCD 파일 읽기
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map2_pcd_file, *cloud_map2) == -1) {
        PCL_ERROR("Couldn't read the PCD file: %s\n", map2_pcd_file.c_str());
        return -1;
    }

    // map2를 map1 기준의 로컬 좌표계로 변환
    for (auto& point : *cloud_map2) {
        // map2의 로컬 좌표를 글로벌 ECEF 좌표로 변환
        Eigen::Vector3d localPoint_map2(point.x, point.y, point.z);
        Eigen::Vector3d globalPoint_map2 = LocalToGlobal(localPoint_map2, originECEF2, heading2);

        // map1 기준의 로컬 좌표계로 변환
        Eigen::Vector3d localPoint_map1 = GlobalToLocal(globalPoint_map2, originECEF1, heading1);
        
        // 변환된 좌표를 다시 PCD에 저장
        point.x = localPoint_map1.x();
        point.y = localPoint_map1.y();
        point.z = localPoint_map1.z();
    }

    // 변환된 포인트 클라우드를 새로운 PCD 파일로 저장
    std::string output_pcd_file = "output.pcd";
    pcl::io::savePCDFileASCII(output_pcd_file, *cloud_map2);
    std::cout << "Saved transformed point cloud to " << output_pcd_file << std::endl;

    return 0;
}
