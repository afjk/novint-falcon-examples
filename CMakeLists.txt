cmake_minimum_required(VERSION 3.10)
project(FalconTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libnifalconのパスを設定
set(LIBNIFALCON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libnifalcon")
set(LIBNIFALCON_BUILD_DIR "${LIBNIFALCON_DIR}/build")

# インクルードディレクトリ
include_directories(
    ${LIBNIFALCON_DIR}/include
    /opt/homebrew/include
)

# ライブラリディレクトリ
link_directories(
    ${LIBNIFALCON_BUILD_DIR}/lib
    /opt/homebrew/lib
)

# 実行ファイルの作成
add_executable(falcon_test falcon_test.cpp)
add_executable(falcon_active_test falcon_active_test.cpp)
add_executable(falcon_gentle_motion falcon_gentle_motion.cpp)
add_executable(falcon_y_motion falcon_y_motion.cpp)
add_executable(falcon_cursor_motion falcon_cursor_motion.cpp)
add_executable(falcon_cursor_xy falcon_cursor_xy.cpp)
add_executable(falcon_cursor_xz falcon_cursor_xz.cpp)
add_executable(falcon_cursor_xz_fixed falcon_cursor_xz_fixed.cpp)
add_executable(falcon_record_play falcon_record_play.cpp)

# ライブラリのリンク
target_link_libraries(falcon_test
    nifalcon
    usb-1.0
)

target_link_libraries(falcon_active_test
    nifalcon
    usb-1.0
)

target_link_libraries(falcon_gentle_motion
    nifalcon
    usb-1.0
)

target_link_libraries(falcon_y_motion
    nifalcon
    usb-1.0
)

target_link_libraries(falcon_cursor_motion
    nifalcon
    usb-1.0
    "-framework CoreGraphics"
    "-framework CoreFoundation"
)

target_link_libraries(falcon_cursor_xy
    nifalcon
    usb-1.0
    "-framework CoreGraphics"
    "-framework CoreFoundation"
)

target_link_libraries(falcon_cursor_xz
    nifalcon
    usb-1.0
    "-framework CoreGraphics"
    "-framework CoreFoundation"
)

target_link_libraries(falcon_cursor_xz_fixed
    nifalcon
    usb-1.0
    "-framework CoreGraphics"
    "-framework CoreFoundation"
)

target_link_libraries(falcon_record_play
    nifalcon
    usb-1.0
)
