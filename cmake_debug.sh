mkdir -p build

pushd build
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../CMakeToolchain.cmake ..
popd
