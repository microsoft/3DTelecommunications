# SOUNDIO_FOUND
# SOUNDIO_INCLUDE_DIR
# SOUNDIO_LIBRARY

find_path(SOUNDIO_INCLUDE_DIR NAMES soundio/soundio.h)

find_library(SOUNDIO_LIBRARY NAMES soundio)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOUNDIO DEFAULT_MSG SOUNDIO_LIBRARY SOUNDIO_INCLUDE_DIR)
