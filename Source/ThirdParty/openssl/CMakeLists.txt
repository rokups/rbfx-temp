# Based on original work by David Manura
# Copyright (C) 2007-2012 LuaDist.
# Copyright (C) 2013 Brian Sidebotham

# Redistribution and use of this file is allowed according to the terms of the
# MIT license.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# This directory holds OpenSSL 4.1.0. It contains the C sources and headers of the upstream crypto, ssl, providers
# and include directories, and the files that the upstream build generates.
# To update it, configure and build OpenSSL in a separate directory with these options:
#   perl Configure linux-generic64 no-asm no-shared no-module no-dso no-async no-tests no-docs no-apps no-engine
#   make libcrypto.a libssl.a
# Copy the generated *.c, *.h and *.inc files into this directory, and rename include/openssl/configuration.h to
# configuration.h.in. In that file, change the BN_LLONG, SIXTY_FOUR_BIT_LONG, SIXTY_FOUR_BIT and THIRTY_TWO_BIT
# lines to #cmakedefine. Make crypto/buildinf.h free of local build data.
# The source lists in crypto/CMakeLists.txt and ssl/CMakeLists.txt are the objects of libcrypto.a and libssl.a.

cmake_minimum_required(VERSION 3.5...3.25)
project(openssl C)

include(CheckTypeSize)
check_type_size("long" LONG_INT)
check_type_size("long long" LONG_LONG_INT)
if("${LONG_INT}" EQUAL 8)
    set(SIXTY_FOUR_BIT_LONG ON)
elseif("${LONG_LONG_INT}" EQUAL 8 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SIXTY_FOUR_BIT ON)
else()
    set(THIRTY_TWO_BIT ON)
    set(BN_LLONG ON)
endif()
configure_file(include/openssl/configuration.h.in ${PROJECT_BINARY_DIR}/include/openssl/configuration.h)

add_subdirectory(crypto)
add_subdirectory(ssl)

install (DIRECTORY include/openssl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ FILES_MATCHING PATTERN *.h)
install (DIRECTORY ${PROJECT_BINARY_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ FILES_MATCHING PATTERN *.h)

install_third_party_libs(ssl crypto)
