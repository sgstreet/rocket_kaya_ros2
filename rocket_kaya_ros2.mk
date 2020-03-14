#
# Copyright (C) 2017 Red Rocket Computing, LLC
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# rocket_kaya_ros2.mk
#
# Created on: Mar 16, 2017
#     Author: Stephen Street (stephen@redrocketcomputing.com)
#

ifeq ($(findstring ${BUILD_ROOT},${CURDIR}),)
include ${PROJECT_ROOT}/tools/makefiles/target.mk
else

all:
	colcon build --build-base ${CURDIR} --install-base ${INSTALL_ROOT}/opt/ros2 --base-path ${SOURCE_DIR} --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=0

clean:
	colcon build --build-base ${CURDIR} --install-base ${INSTALL_ROOT}/opt/ros2 --base-path ${SOURCE_DIR} --symlink-install --merge-install --cmake-target clean
	
distclean:
	@echo "DISTCLEAN ${CURDIR}"
	rm -rf ${CURDIR} ${PROJECT_ROOT}/log ${IMAGE_ROOT}/opt/ros2

install:

endif
