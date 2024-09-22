#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Dense>
#include <chrono>  // 시간 측정을 위한 라이브러리

// PCD 파일을 불러오는 함수
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("파일을 불러올 수 없습니다. 파일 경로를 확인하세요: %s\n", filename.c_str());
        return nullptr;
    }

    std::cout << "PCD 파일 로드 완료: " << cloud->points.size() << " 포인트" << std::endl;
    return cloud;
}

// 주어진 좌표 범위 내에 있는 포인트를 필터링하는 함수
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsInGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float min_x, float max_x, float min_y, float max_y) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud->points) {
        if (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y) {
            filtered_cloud->points.push_back(point);
        }
    }

    return filtered_cloud;
}

// 주어진 3개의 점으로 평면 방정식의 계수(A, B, C)를 계산하는 함수
Eigen::Vector4f calculatePlaneEquation(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3) {
    Eigen::Vector3f v1 = p2 - p1;
    Eigen::Vector3f v2 = p3 - p1;
    Eigen::Vector3f normal = v1.cross(v2); // 법선 벡터 계산

    // 평면 방정식의 A, B, C 계산
    float A = normal[0];
    float B = normal[1];
    float C = normal[2];
    float D = -(A * p1[0] + B * p1[1] + C * p1[2]); // D는 평면 방정식 상수항

    return Eigen::Vector4f(A, B, C, D);
}

// 평면 상에서 주어진 x, y에 대응하는 z값을 계산하는 함수
double calculateZOnPlane(const Eigen::Vector4f& plane_coeff, float x, float y) {
    // z = -(A * x + B * y + D) / C
    if (plane_coeff[2] == 0) return 0; // C가 0인 경우 (평면이 수평) 예외 처리
    return -(plane_coeff[0] * x + plane_coeff[1] * y + plane_coeff[3]) / plane_coeff[2];
}

// 하나의 구역에 대한 부피 계산 함수
double calculateVolumeForGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    double grid_area, 
    const Eigen::Vector4f& bottom_plane, 
    const std::vector<Eigen::Vector4f>& side_planes) {

    double total_volume = 0.0;

    for (const auto& point : cloud->points) {
        // 윗면 포인트 클라우드에서 z값 사용
        double upper_z = point.z;

        // 아랫면의 z값 계산
        double lower_z = calculateZOnPlane(bottom_plane, point.x, point.y);

        // 각 측면에 대해 z값 보정
        for (const auto& side_plane : side_planes) {
            double side_z = calculateZOnPlane(side_plane, point.x, point.y);
            lower_z = std::max(lower_z, side_z); // 측면의 z값이 더 클 경우 lower_z 업데이트
        }

        // 부피는 윗면과 아랫면의 z 차이에 기반해 계산
        double height_diff = upper_z - lower_z;
        if (height_diff > 0) {
            total_volume += height_diff * grid_area; // 각 구역의 부피
        }
    }

    return total_volume;
}

int main(int argc, char** argv) {
    // PCD 파일 경로 입력
    if (argc < 2) {
        std::cerr << "사용법: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    // 시간 측정을 시작
    auto start_time = std::chrono::high_resolution_clock::now();

    // PCD 파일 로드
    std::string filename = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPointCloud(filename);
    if (!cloud) {
        return -1;
    }

    // 타겟 윗면의 4개 좌표 (사용자가 입력한 좌표)
    std::vector<Eigen::Vector3f> upper_bounds = {
        {5.625, 5.625, 0}, 
        {-5.625, 5.625, 0},
        {-5.625, -5.625, 0},
        {5.625, -5.625, 0}
    };

    // 타겟 아랫면의 4개 좌표 (사용자가 입력한 좌표)
    std::vector<Eigen::Vector3f> lower_bounds = {
        {5.1114, 3.8158, -2.4475},
        {-2.3886, 3.8158, -2.4475},
        {-2.3886, -3.6842, -2.4475},
        {5.1114, -3.6842, -2.4475}
    };

    // 아랫면 평면 방정식 계산 (3개의 점 사용)
    Eigen::Vector4f bottom_plane = calculatePlaneEquation(lower_bounds[0], lower_bounds[1], lower_bounds[2]);

    // 측면 평면 방정식 계산 (윗면과 아랫면의 대응하는 점을 사용)
    std::vector<Eigen::Vector4f> side_planes = {
        calculatePlaneEquation(lower_bounds[0], lower_bounds[1], upper_bounds[0]), // 첫 번째 측면
        calculatePlaneEquation(lower_bounds[1], lower_bounds[2], upper_bounds[1]), // 두 번째 측면
        calculatePlaneEquation(lower_bounds[2], lower_bounds[3], upper_bounds[2]), // 세 번째 측면
        calculatePlaneEquation(lower_bounds[3], lower_bounds[0], upper_bounds[3])  // 네 번째 측면
    };

    // 윗면의 x, y 범위 계산 (PCD 포인트 클라우드를 사용)
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    
    for (const auto& point : cloud->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
    }

    // 그리드로 나누기 위한 간격 정의 (예시로 1미터 간격)
    float grid_size = 0.10;

    // 전체 부피를 저장할 변수
    double total_volume = 0.0;

    // 그리드 구역별로 필터링하여 부피를 계산
    for (float x = min_x; x < max_x; x += grid_size) {
        for (float y = min_y; y < max_y; y += grid_size) {
            // 현재 그리드의 x, y 범위 정의
            float grid_min_x = x;
            float grid_max_x = x + grid_size;
            float grid_min_y = y;
            float grid_max_y = y + grid_size;

            // 현재 그리드 내의 포인트 필터링
            pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud = filterPointsInGrid(cloud, grid_min_x, grid_max_x, grid_min_y, grid_max_y);

            // 그리드 면적 계산 (단순 사각형으로 가정)
            double grid_area = grid_size * grid_size;

            // 그리드 내 포인트들에 대한 부피 계산
            double grid_volume = calculateVolumeForGrid(grid_cloud, grid_area, bottom_plane, side_planes);

            // 구역 부피를 전체 부피에 추가
            total_volume += grid_volume;
        }

    }

    // 시간 측정을 종료
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;

    // 전체 부피 출력
    std::cout << "Grid size: " << grid_size << " size" << std::endl;
    std::cout << "계산된 전체 부피: " << total_volume << " cubic meters" << std::endl;
    std::cout << "계산에 걸린 시간: " << duration.count() << " seconds" << std::endl;

    return 0;
}
