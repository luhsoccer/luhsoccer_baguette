# tests {#tests}

This folder contains all tests for the software. The test framework is `gtest` from Google. To add a test simply create a new `*.cpp` file with the new test or add the test to an existing file if applicable. For documentation on how to write tests please refer to the [gtest Documentation](https://google.github.io/googletest/).

## Folder structure

```txt
tests/
├─ module-1/
│  ├─ test_class_1.cpp
│  ├─ test_class_2.cpp
├─ module-2/
│  ├─ test_class_x.cpp
├─ CMakeLists.txt
```

As you can see each module should have their own sub-directory. Also each class of the module should have it's own `*.cpp` file, which contains the tests. Currently only unit testing is supported.

To add a new module into the test-suit simply add the module target into the `target_link_libraries(...)` function in the `CMakeLists.txt` file.
