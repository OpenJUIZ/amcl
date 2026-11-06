# amcl

AMCL (Adaptive Monte Carlo Localization) library taken from the ROS
[navigation](https://github.com/ros-planning/navigation) stack, without any ROS dependency.
It was originally part of the Player project.

It contains the particle filter (`pf`), the occupancy map with its likelihood field (`map`)
and the odometry and laser models (`sensors`). The ROS node itself is not included; see
[OpenJUIZ/localization_amcl](https://github.com/OpenJUIZ/localization_amcl) for a JUIZ
component built on this library.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

Linux, macOS and Windows (MSVC) are built by CI.

As a subproject:

```cmake
add_subdirectory(third_party/amcl)
target_link_libraries(your_target PRIVATE amcl::amcl)
```

Tests and install rules are only enabled when this is the top-level project
(`AMCL_BUILD_TESTS`, `AMCL_INSTALL`).

## Notes

- Random numbers come from the global `drand48()` state (a `rand()`-based replacement on
  systems without it). `pf_alloc` seeds it from the clock; `pf_init` reseeds it. The library
  is not thread-safe: guard a filter and its sensors with one lock.
- `map_t::origin_x`/`origin_y` is the world position of the **centre** of the map, as the ROS
  node sets it (`origin + size / 2 * scale` of a map_server map).

## License

LGPL (the source headers say LGPL-2.1 or later; `LICENSE` holds the LGPL-3.0 text).
