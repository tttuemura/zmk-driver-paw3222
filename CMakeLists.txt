if(CONFIG_PAW3222)
    zephyr_library()
    zephyr_library_sources(src/paw3222.c)

    zephyr_library_sources_ifdef(CONFIG_PAW3222_SCROLL_ACCEL src/paw3222_scroll_accel.c)

    zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
endif()
