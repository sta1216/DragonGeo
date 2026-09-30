# 安装包消费方冒烟工程。
#
# 它与仓库内的构建完全独立：不做 add_subdirectory，只用 find_package 找到
# 已安装的 DragonGeo，因此能测到「安装导出的目标是否自带使用要求」这件事 ——
# 仓库内的测试与示例都在同一棵 CMake 树里，会掩盖导出时漏掉的选项。
#
# 这里**刻意**不设置 /utf-8。公共头是 UTF-8 无 BOM，编码选项必须由
# DragonGeo::DragonGeo 以 INTERFACE 形式带到消费方；若由消费方自己补，这个
# 冒烟测试就会失去意义。
cmake_minimum_required(VERSION 3.20)

project(DragonGeoConsumer LANGUAGES CXX)

find_package(DragonGeo REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE DragonGeo::DragonGeo)

# /WX：警告即错误。**这道防线不能省。** MSBuild 默认按 `/W1 /WX-` 编译，且
# 安装前缀位于 `/external:I` 之下、外部头的警告被 `/external:W0` 压到 0 级 ——
# 缺了 /utf-8 时这里只产生 C4819 警告、0 个 error，作业只看退出码就会静默通过
# （已实测）。/WX 把 C4819 变成硬失败，回归才守得住（实测：`cmake --build`
# 退出码 1，诊断是 C2220「警告被视为错误」+ C4819）。
#
# **触发点是本工程的 `main.cpp`，不是 DragonGeo 的头文件**：负对照里唯一的 C4819
# 指向 `main.cpp` 按 UTF-8 写的中文注释，头文件的 C4819 已被上面的外部头机制
# 压掉。所以**前提是 `main.cpp` 保持非 ASCII** —— 若把它改成纯 ASCII，`/WX`
# 就再也接不到任何警告，这道防线会**静默失效**。
# 注意：这里**仍然**不设置任何编码选项，见文件开头。
if(MSVC)
    target_compile_options(consumer PRIVATE /WX)
endif()
