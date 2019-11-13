#
# Try to find OPENVOLUMEMESH
# Once done this will define
#
# OPENVOLUMEMESH_FOUND           - system has OPENVOLUMEMESH
# OPENVOLUMEMESH_INCLUDE_DIR     - the OPENVOLUMEMESH include directory
# OPENVOLUMEMESH_LIBRARY         - Link these to use OPENVOLUMEMESH
# OPENVOLUMEMESH_LIBRARY_DIR     - Library DIR of OPENVOLUMEMESH
#
# Copyright 2013 Computer Graphics Group, RWTH Aachen University
# Authors: Jan Möbius <moebius@cs.rwth-aachen.de>
#          Hans-Christian Ebke <ebke@cs.rwth-aachen.de>
#          Max Lyon <lyon@cs.rwth-aachen.de>
#
# This file is part of HexEx.
#
# HexEx is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# HexEx is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with HexEx.  If not, see <http://www.gnu.org/licenses/>.
#

IF (OPENVOLUMEMESH_INCLUDE_DIR)
    # Already in cache, be silent
    SET(OPENVOLUMEMESH_FIND_QUIETLY TRUE)
ENDIF (OPENVOLUMEMESH_INCLUDE_DIR)

FIND_PATH(OPENVOLUMEMESH_INCLUDE_DIR OpenVolumeMesh/Mesh/TetrahedralMesh.hh
        env OpenVolumeMesh_DIR/include
        PATHS /usr/local/include
        /usr/include
        /usr/local/OpenVolumeMesh/include
        /ACG/acgdev/gcc-4.0-x86_64/OVM/OpenVolumeMesh/installed/include
        "C:\\Program Files\\OpenVolumeMesh\\include"
        )

IF (OPENVOLUMEMESH_INCLUDE_DIR )
    message(${OPENVOLUMEMESH_INCLUDE_DIR})
    SET(OPENVOLUMEMESH_FOUND TRUE)
    ADD_DEFINITIONS(-DENABLE_OPENVOLUMEMESH)
    find_library(OPENVOLUMEMESHD_LIB NAME OpenVolumeMeshd
            PATHS $ENV{OpenVolumeMesh_DIR}/lib NO_DEFAULT_PATH)
    find_library(OPENVOLUMEMESH_LIB NAME OpenVolumeMesh
            PATHS $ENV{OpenVolumeMesh_DIR}/lib NO_DEFAULT_PATH)
    set(OPENVOLUMEMESH_LIBRARY
            debug ${OPENVOLUMEMESHD_LIB}
            optimized ${OPENVOLUMEMESH_LIB})
ELSE (OPENVOLUMEMESH_INCLUDE_DIR)
    SET(OPENVOLUMEMESH_FOUND FALSE )
ENDIF (OPENVOLUMEMESH_INCLUDE_DIR )
