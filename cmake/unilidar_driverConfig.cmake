@PACKAGE_INIT@

include(CMakeFindDependencyMacro)

# Find dependencies if needed
# find_dependency(SomeDependency REQUIRED)

# Include the targets file
include("${CMAKE_CURRENT_LIST_DIR}/unilidar_driverTargets.cmake")

# Set variables for backward compatibility
set(unilidar_driver_LIBRARIES unilidar_driver::unilidar_driver)
set(unilidar_driver_INCLUDE_DIRS "@PACKAGE_INCLUDE_INSTALL_DIR@")

check_required_components(unilidar_driver)