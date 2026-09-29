# 安装包消费方冒烟工程。
#
# 它与仓库内的构建完全独立：不做 add_subdirectory，只用 find_package 找到
# 已安装的 GeoCore，因此能测到「安装导出的目标是否自带使用要求」这件事 ——
# 仓库内的测试与示例都在同一棵 CMake 树里，会掩盖导出时漏掉的选项。
#
# 这里**刻意**不设置 /utf-8。公共头是 UTF-8 无 BOM，编码选项必须由
# GeoCore::GeoCore 以 INTERFACE 形式带到消费方；若由消费方自己补，这个
# 冒烟测试就会失去意义。
cmake_minimum_required(VERSION 3.20)

project(GeoCoreConsumer LANGUAGES CXX)

find_package(GeoCore REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE GeoCore::GeoCore)
