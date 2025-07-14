

target-y = \
	$(subst .c, , $(wildcard *-??.c))

define test-defaults
	$1_files-y = \
		$(addsuffix .c, $1) \
		$(subst main.c,,$(wildcard ../src/*.c))

	$1_includes-y = \
		${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/include \
		../src/

	$1_libraries-y = \
		${CLEW_3RDPARTY_INSTALL_PATH}/usr/local/lib

	$1_../src/input-osm-pbf-fileformat.pb-c.c_cflags-y += \
		-Wno-missing-braces

	$1_ldflags-y = \
		-lprotobuf-c \
		-lz \
		-lm
endef

$(eval $(foreach T,$(target-y), $(eval $(call test-defaults,$T))))

include ${CLEW_LIBMAKEFILE_PATH}/Makefile.lib
