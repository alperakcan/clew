
target.extra-y = \
	install

include ${CLEW_LIBMAKEFILE_PATH}/Makefile.lib

patch:
	${Q}if [ ! -e .patch ]; then \
		( \
		  	true \
		) && \
		touch .patch; \
	fi

config: patch
	${Q}if [ ! -e .config ]; then \
		( \
			mkdir -p ../protobuf-c/build && \
			cd ../protobuf-c/build && \
			CFLAGS=-g \
			LDFLAGS= \
			MAKEFLAGS= \
			cmake \
				-Wno-dev \
				-DCMAKE_BUILD_TYPE=Release \
				-DBUILD_SHARED_LIBS=OFF \
				-DBUILD_PROTOC=OFF \
				../build-cmake \
		) && \
		touch .config; \
		rm -f .build ; \
	fi

build: config
	${Q}if [ ! -e .build ]; then \
		( \
			mkdir -p ../protobuf-c/build && \
			cd ../protobuf-c/build && \
			make \
		) && \
		touch .build; \
		rm -f .install; \
	fi

install: build
	${Q}if [ ! -e .install ]; then \
		( \
			mkdir -p ../protobuf-c/build && \
			cd ../protobuf-c/build && \
			DESTDIR=${CLEW_3RDPARTY_INSTALL_PATH} make install \
		) && \
		touch .install; \
	fi

clean:
	${Q}${RM} .patch
	${Q}${RM} ../protobuf-c/build
	${Q}${RM} .config
	${Q}${RM} .build
	${Q}${RM} .install
