#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <cmath>

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

// 평면 방정식을 계산하는 함수
Eigen::Vector4f calculatePlaneEquation(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3) {
    Eigen::Vector3f v1 = p2 - p1;
    Eigen::Vector3f v2 = p3 - p1;
    Eigen::Vector3f normal = v1.cross(v2);
    float A = normal[0];
    float B = normal[1];
    float C = normal[2];
    float D = -(A * p1[0] + B * p1[1] + C * p1[2]);
    return Eigen::Vector4f(A, B, C, D);
}

// 주어진 평면 방정식에 따라 z값을 계산하는 함수
double calculateZOnPlane(const Eigen::Vector4f& plane_coeff, float x, float y) {
    if (plane_coeff[2] == 0) return 0;
    return -(plane_coeff[0] * x + plane_coeff[1] * y + plane_coeff[3]) / plane_coeff[2];
}

// 좌우, 위아래의 값을 기반으로 선형 보간법 적용
double interpolateZValue(const std::map<std::pair<int, int>, std::pair<double, int>>& grid_map,
                         int grid_x_index, int grid_y_index, int grid_count_x, int grid_count_y) {
    double left_z = 0.0, right_z = 0.0, top_z = 0.0, bottom_z = 0.0;
    bool has_left = false, has_right = false, has_top = false, has_bottom = false;

    // 좌우 확인
    auto left_it = grid_map.find({grid_x_index - 1, grid_y_index});
    if (left_it != grid_map.end()) {
        left_z = left_it->second.first / left_it->second.second;
        has_left = true;
    }

    auto right_it = grid_map.find({grid_x_index + 1, grid_y_index});
    if (right_it != grid_map.end()) {
        right_z = right_it->second.first / right_it->second.second;
        has_right = true;
    }

    // 위아래 확인
    auto top_it = grid_map.find({grid_x_index, grid_y_index + 1});
    if (top_it != grid_map.end()) {
        top_z = top_it->second.first / top_it->second.second;
        has_top = true;
    }

    auto bottom_it = grid_map.find({grid_x_index, grid_y_index - 1});
    if (bottom_it != grid_map.end()) {
        bottom_z = bottom_it->second.first / bottom_it->second.second;
        has_bottom = true;
    }

    // 보간법 적용
    if (has_left && has_right) {
        return (left_z + right_z) / 2.0;  // 좌우 중간값
    } else if (has_left) {
        return left_z;
    } else if (has_right) {
        return right_z;
    } else if (has_top && has_bottom) {
        return (top_z + bottom_z) / 2.0;  // 위아래 중간값
    } else if (has_top) {
        return top_z;
    } else if (has_bottom) {
        return bottom_z;
    }

    return 0.0;  // 주변에 값이 없을 경우
}

// 그리드별 부피 계산 함수
double calculateVolumeForGrid(const std::map<std::pair<int, int>, std::pair<double, int>>& grid_map, 
                              double grid_area, const Eigen::Vector4f& bottom_plane, 
                              const std::vector<Eigen::Vector4f>& side_planes, float grid_min_x, float grid_min_y, 
                              float grid_size, int grid_x_index, int grid_y_index, int grid_count_x, int grid_count_y) {

    // 윗면의 z값 계산
    auto it = grid_map.find({grid_x_index, grid_y_index});
    double upper_z = 0.0;

    if (it != grid_map.end()) {
        upper_z = it->second.first / it->second.second;  // z값 평균 사용
    } else {
        // 인접한 그리드 셀의 z값을 사용해 보간된 z값을 계산
        upper_z = interpolateZValue(grid_map, grid_x_index, grid_y_index, grid_count_x, grid_count_y);
    }

    // 아랫면의 z값 계산 (평면 방정식을 기반으로)
    double lower_z = calculateZOnPlane(bottom_plane, grid_min_x, grid_min_y);

    // 각 측면의 z값 보정
    for (const auto& side_plane : side_planes) {
        double side_z = calculateZOnPlane(side_plane, grid_min_x, grid_min_y);
        lower_z = std::max(lower_z, side_z);
    }

    double height_diff = upper_z - lower_z;
    if (height_diff > 0) {
        return height_diff * grid_area;  // 부피 계산
    }
    return 0.0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "사용법: " << argv[0] << " <input_pcd_file>" << std::endl;
        return -1;
    }

    // 시간 측정을 시작
    auto start_time = std::chrono::high_resolution_clock::now();

    std::string filename = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPointCloud(filename);
    if (!cloud) return -1;

    // 타겟 윗면의 좌표들
    std::vector<Eigen::Vector3f> upper_bounds = {
        {5.625, 5.625, 0}, 
        {-5.625, 5.625, 0},
        {-5.625, -5.625, 0},
        {5.625, -5.625, 0}
    };

    // 타겟 아랫면의 좌표들
    std::vector<Eigen::Vector3f> lower_bounds = {
        {5.1114, 3.8158, -2.4475},
        {-2.3886, 3.8158, -2.4475},
        {-2.3886, -3.6842, -2.4475},
        {5.1114, -3.6842, -2.4475}
    };

    // 아랫면 평면 방정식 계산
    Eigen::Vector4f bottom_plane = calculatePlaneEquation(lower_bounds[0], lower_bounds[1], lower_bounds[2]);

    // 측면 평면 방정식 계산
    std::vector<Eigen::Vector4f> side_planes = {
        calculatePlaneEquation(lower_bounds[0], lower_bounds[1], upper_bounds[0]), 
        calculatePlaneEquation(lower_bounds[1], lower_bounds[2], upper_bounds[1]),
        calculatePlaneEquation(lower_bounds[2], lower_bounds[3], upper_bounds[2]),
        calculatePlaneEquation(lower_bounds[3], lower_bounds[0], upper_bounds[3])
    };

    float grid_size = 0.1;  // 그리드 간격
    std::map<std::pair<int, int>, std::pair<double, int>> grid_map;

    // x, y 방향 그리드 수 계산
    float min_x = std::min({upper_bounds[0][0], upper_bounds[1][0], upper_bounds[2][0], upper_bounds[3][0]});
    float max_x = std::max({upper_bounds[0][0], upper_bounds[1][0], upper_bounds[2][0], upper_bounds[3][0]});
    float min_y = std::min({upper_bounds[0][1], upper_bounds[1][1], upper_bounds[2][1], upper_bounds[3][1]});
    float max_y = std::max({upper_bounds[0][1], upper_bounds[1][1], upper_bounds[2][1], upper_bounds[3][1]});

    // 그리드 맵에 각 포인트를 할당
    for (const auto& point : cloud->points) {
        if (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y) {
            int grid_x = static_cast<int>(std::floor((point.x - min_x) / grid_size));
            int grid_y = static_cast<int>(std::floor((point.y - min_y) / grid_size));
            auto grid_coord = std::make_pair(grid_x, grid_y);
            grid_map[grid_coord].first += point.z;  // z값 합산
            grid_map[grid_coord].second += 1;       // 포인트 개수 증가
        }
    }

    double total_volume = 0.0;
    int grid_count_x = static_cast<int>(std::ceil((max_x - min_x) / grid_size));
    int grid_count_y = static_cast<int>(std::ceil((max_y - min_y) / grid_size));

    // 그리드별로 부피 계산
    for (int i = 0; i < grid_count_x; ++i) {
        for (int j = 0; j < grid_count_y; ++j) {
            float grid_min_x = min_x + i * grid_size;
            float grid_min_y = min_y + j * grid_size;
            double grid_area = grid_size * grid_size;
            double grid_volume = calculateVolumeForGrid(grid_map, grid_area, bottom_plane, side_planes, grid_min_x, grid_min_y, grid_size, i, j, grid_count_x, grid_count_y);
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
