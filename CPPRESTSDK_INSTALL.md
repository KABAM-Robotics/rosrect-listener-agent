# cpprestsdk Windows Installation for error_resolution_diagnoser

The `error_resolution_diagnoser` package uses Microsoft's `cpprestsdk` library as a dependency for REST API communication and JSON construction/parsing. On Windows, the latest version at the time of this package's development (`2.10`) did not work out of the box because of the platform dependent string parsing in the package. Fortunately, one of the users has a public fork where they have provided support for narrow strings on Windows. Instead of downloading the dependency using `vcpkg` directly from source, we will build it manually source and then install it as follows:

1. It is recommended to use Windows PowerShell or a Visual Studio Developer command prompt, in the administrator mode for the following steps.

2. Clone the public fork of `cpprestsdk`, https://github.com/jason-ha/cpprestsdk.git in a folder of your choice:
    
        PS> cd c:\opt
        PS> git clone https://github.com/jason-ha/cpprestsdk.git

4. Checkout to the `win32_narrow_strings` branch that has the necessary fix:
    
        PS> cd cpprestsdk
        PS> git checkout win32_narrow_strings

5. Now, follow the instructions in [this page](https://github.com/microsoft/cpprestsdk/wiki/How-to-build-for-Windows#build-from-source) to use `CMake` and `vcpkg` to build from source on Windows. The built binaries will be in `<build_dir>\Binaries\Release` or `<build_dir>\Binaries\Debug` depending on the configuration you build with.

6. Create a new folder where you would like to install the package you just built:
    
        PS> mkdir c:\opt\cpprestsdk_install

7. Install the package using `cmake install` from the build directory `<build_dir>`:
    
        PS> cd c:\opt\cpprestsdk\<build_dir>\  
        PS> cmake --install . --prefix "C:\opt\cpprestsdk_install"

8. Add `C:\opt\cpprestsdk_install\bin` to the Windows System path by editing the `Path` variable.

9. In [CmakeLists.txt](CmakeLists.txt), edit line 14 to set the `cpprestsdk_DIR` to the appropriate value such as follows that contains the necessary `*.cmake` files:
    
        set(cpprestsdk_DIR C:/opt/cpprestsdk_install/lib/cpprestsdk/)
