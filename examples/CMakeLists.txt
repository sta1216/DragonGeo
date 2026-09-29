add_executable(hello_geocore hello_geocore.cpp)
target_link_libraries(hello_geocore PRIVATE GeoCore::GeoCore)

add_executable(vector_basics vector_basics.cpp)
target_link_libraries(vector_basics PRIVATE GeoCore::GeoCore)

add_executable(transform_pipeline transform_pipeline.cpp)
target_link_libraries(transform_pipeline PRIVATE GeoCore::GeoCore)
