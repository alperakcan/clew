
subdir-y += \
	clipper2-build \
	rapidjson-build \
	shapelib-build \
	protobuf-c-build

subdir-${CLEW_MIMALLOC_ENABLE} += \
	mimalloc-build

include ${CLEW_LIBMAKEFILE_PATH}/Makefile.lib

clean:
	${Q}${RM} ${CLEW_3RDPARTY_INSTALL_PATH}
