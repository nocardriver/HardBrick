cmake_minimum_required(VERSION 3.22)

#
# App layer: aggregates all application modules.
# A new module is registered by adding its subdirectory here and
# appending its target name to APP_LIBS below.
#

# Register modules
add_subdirectory(gps)
add_subdirectory(display)

# Export list of application libraries to the parent (top-level) scope
set(APP_LIBS
    gps
    display
    # Add new application library targets here
    PARENT_SCOPE
)
