#script for building opencv
TOOL_CHAIN_FILE="/home/zhufan/Android/Sdk/ndk-bundle/build/cmake/android.toolchain.cmake"
ABI="armeabi-v7a"
OUTPUT_PATH="/home/zhufan/ORB_SLAM/app/libs/armeabi-v7a"
EIGEN_DIR="/home/zhufan"
OPENCV_DIR="/home/zhufan/opencv-android/sdk/native/jni"
BUILD_TYPE=Release

mkdir build
cd build

cmake ../ \
    -DEIGEN3_INCLUDE_DIR=${EIGEN_DIR} \
    -DOpenCV_DIR=${OPENCV_DIR} \
    -DANDROID_ABI=${ABI} \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DANDROID_ARM_NEON=TURE \
    -DCMAKE_TOOLCHAIN_FILE=${TOOL_CHAIN_FILE} \
    -DANDROID_NATIVE_API_LEVEL=14 \
    -DANDROID_TOOLCHAIN=clang \
    -DANDROID_LINKER_FLAGS="-landroid -llog" \
    -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${OUTPUT_PATH}

make


