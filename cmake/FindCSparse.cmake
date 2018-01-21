# Look for csparse; note the difference in the directory specifications!
# Find g2o bundle CSparse
FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  ${G2O_ROOT}/include/EXTERNAL/csparse
  /usr/include/suitesparse
  /usr/include
  /opt/local/include
  /usr/local/include
  /sw/include
  /usr/include/ufsparse
  /opt/local/include/ufsparse
  /usr/local/include/ufsparse
  /sw/include/ufsparse
  )

FIND_LIBRARY(CSPARSE_LIBRARY NAMES cxsparse g2o_ext_csparse
  PATHS
  ${G2O_ROOT}/lib
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CSPARSE DEFAULT_MSG
  CSPARSE_INCLUDE_DIR CSPARSE_LIBRARY)
