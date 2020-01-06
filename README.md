# Sensor Fusion Self-Driving Car Course

### building on win32/64 using vcpkg and VSCode:
* get a complier. Here is a list of VSCode supported ones [https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
	* I used msvc and followed this guide [https://code.visualstudio.com/docs/cpp/config-msvc](https://code.visualstudio.com/docs/cpp/config-msvc)
* get [https://github.com/microsoft/vcpkg](https://github.com/microsoft/vcpkg)
	* `vcpkg install pcl` and `vcpkg install boost`. Add the :x64-windows flag if needed (e.g. `vcpkg install pcl:x64-windows`). For some reason, all of boost doesn't come with PCL install command.
* build project using CMAKE gui
	* get cmake gui [https://cmake.org/runningcmake/](https://cmake.org/runningcmake/)
	* configure project using toolset "${vcpkgRoot}/scripts/buildsystems/vcpkg.cmake"
		* Reference [https://developerpaul123.github.io/c++/cmake/using-vcpkg-on-windows/](https://developerpaul123.github.io/c++/cmake/using-vcpkg-on-windows/)
	* run `cmake --build .` inside the build dir
* set up VSCode:
	* get Microsoft's C/C++ extension.
	* Command Palette: C/C++: Edit Configurations (UI)
		* be sure the vcpkg directory is included `${vcpkgRoot}/installed/x64-windows/include`
	* add `cmake --build .` to tasks.json
	* add `${workspaceFolder}/build/Debug/environment.exe` to launch.json for easy debugging
* shortcuts:
	* `ctrl + shift + b` to build
	* `F5` to debug

### project references
* PCL RANSAC tutorial [http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices](http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices) - was helpful in defining the segmentation algorithm to retrieve important indicies, then filtering those indecies to extract useful point clouds.
* PCL KD Clustering tutorial [http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php](http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php) - was helpful in separating entities in the scene

### original repo
check out the original readme at [https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/README.md](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/README.md)
