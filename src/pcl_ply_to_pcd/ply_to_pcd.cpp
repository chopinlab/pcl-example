#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "사용법: " << argv[0] << " <input.ply> <output.pcd>" << std::endl;
        return -1;
    }

    std::string input_filename = argv[1];   // 입력 PLY 파일 경로
    std::string output_filename = argv[2];  // 출력 PCD 파일 경로

    // 포인트 클라우드 생성
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PLY 파일 로드
    if (pcl::io::loadPLYFile(input_filename, *cloud) == -1) {
        PCL_ERROR("PLY 파일을 불러올 수 없습니다. 파일 경로를 확인하세요: %s\n", input_filename.c_str());
        return -1;
    }

    std::cout << "PLY 파일 로드 완료: " << cloud->points.size() << " 포인트" << std::endl;

    // PCD 파일로 저장
    if (pcl::io::savePCDFileASCII(output_filename, *cloud) == -1) {
        PCL_ERROR("PCD 파일을 저장할 수 없습니다. 경로를 확인하세요: %s\n", output_filename.c_str());
        return -1;
    }

    std::cout << "PCD 파일로 저장 완료: " << output_filename << std::endl;

    return 0;
}
