
target-y += \
	clew

ifeq (${CLEW_MIMALLOC_ENABLE}, y)

clew_files-${__LINUX__} += \
	${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/lib/mimalloc-2.2/mimalloc.o

clew_files-${__DARWIN__} += \
	${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/lib/mimalloc-2.2/mimalloc.o

clew_files-${__WINDOWS__} += \
	${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/lib/mimalloc-2.2/mimalloc.obj

endif

clew_files-y += \
	debug.c \
	input-osm-pbf-fileformat.pb-c.c \
	input-osm-pbf-osmformat.pb-c.c \
	input-osm-pbf.c \
	input.c \
	stack.c \
	tag.c \
	main.c \

clew_includes-y += \
	${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/include

clew_libraries-y += \
	${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/lib

clew_depends-y += \
	input-osm-pbf-fileformat.pb-c.c \
	input-osm-pbf-fileformat.pb-c.h \
	input-osm-pbf-osmformat.pb-c.c \
	input-osm-pbf-osmformat.pb-c.h \

clew_input-osm-pbf-fileformat.pb-c.c_cflags-y += \
	-Wno-missing-braces

clew_cxxflags-y += \
	-std=c++20 \
	-fno-exceptions

clew_ldflags-y += \
	-lprotobuf-c \
	-lz

dist.dir        = ../dist
dist.base       = clew
dist.bin-y     += clew

include ${CLEW_LIBMAKEFILE_PATH}/Makefile.lib

input-osm-pbf-fileformat.pb-c.c input-osm-pbf-fileformat.pb-c.h: input-osm-pbf-fileformat.proto
	${Q}echo "  GEN        $*"
	${Q}protoc-c input-osm-pbf-fileformat.proto --c_out=.

input-osm-pbf-osmformat.pb-c.c input-osm-pbf-osmformat.pb-c.h: input-osm-pbf-osmformat.proto
	${Q}echo "  GEN        $*"
	${Q}protoc-c input-osm-pbf-osmformat.proto --c_out=.

tag.c: tag*.h Makefile tag.sh
	${Q}printf "  GEN        `pwd`/$@\n"
	${Q}sh ./tag.sh ./ > tag.c

clean:
	${Q}${RM} input-osm-pbf-fileformat.pb-c.c input-osm-pbf-fileformat.pb-c.h
	${Q}${RM} input-osm-pbf-osmformat.pb-c.c input-osm-pbf-osmformat.pb-c.h
	${Q}${RM} tag.c
