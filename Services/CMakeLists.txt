cmake_minimum_required(VERSION 3.22)

#
# Services layer: aggregates all service modules.
# A new module is registered by adding its subdirectory here and
# appending its target name to SERVICES_LIBS below.
#

# Register modules
add_subdirectory(SEGGER_RTT)

# Export list of service libraries to the parent (top-level) scope
set(SERVICES_LIBS
    segger_rtt
    # Add new service library targets here
    PARENT_SCOPE
)
