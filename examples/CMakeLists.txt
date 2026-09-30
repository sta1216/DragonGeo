add_executable(HelloDragonGeo HelloDragonGeo.cpp)
target_link_libraries(HelloDragonGeo PRIVATE DragonGeo::DragonGeo)

add_executable(VectorBasics VectorBasics.cpp)
target_link_libraries(VectorBasics PRIVATE DragonGeo::DragonGeo)

add_executable(TransformPipeline TransformPipeline.cpp)
target_link_libraries(TransformPipeline PRIVATE DragonGeo::DragonGeo)
