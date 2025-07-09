
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
			mkdir -p ../shapelib/build && \
			cd ../shapelib/build && \
			CFLAGS=-g \
			LDFLAGS= \
			MAKEFLAGS= \
			cmake \
				-Wno-dev \
				-DCMAKE_BUILD_TYPE=Release \
				-DBUILD_SHARED_LIBS=OFF \
				-DBUILD_TESTING=OFF \
				.. \
		) && \
		touch .config; \
		rm -f .build ; \
	fi

build: config
	${Q}if [ ! -e .build ]; then \
		( \
			mkdir -p ../shapelib/build && \
			cd ../shapelib/build && \
			make \
		) && \
		touch .build; \
		rm -f .install; \
	fi

install: build
	${Q}if [ ! -e .install ]; then \
		( \
			mkdir -p ../shapelib/build && \
			cd ../shapelib/build && \
			DESTDIR=${CLEW_3RDPARTY_INSTALL_PATH} make install \
		) && \
		touch .install; \
	fi

clean:
	${Q}${RM} .patch
	${Q}${RM} ../shapelib/build
	${Q}${RM} .config
	${Q}${RM} .build
	${Q}${RM} .install
