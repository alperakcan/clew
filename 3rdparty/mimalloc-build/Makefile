
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
	      rm -rf ../mimalloc/build && \
	      mkdir -p ../mimalloc/build && \
	      cd ../mimalloc/build && \
	      CFLAGS=-g \
	      LDFLAGS= \
	      MAKEFLAGS= \
	      cmake \
	      	-DCMAKE_BUILD_TYPE=Release \
	      	-DMI_BUILD_SHARED=OFF \
	        -DMI_BUILD_STATIC=ON \
		-DMI_BUILD_OBJECT=ON \
		-DMI_BUILD_TESTS=OFF \
	        .. \
	  ) && \
	  touch .config; \
	  rm -f .build ; \
	fi

build: config
	${Q}if [ ! -e .build ]; then \
	  ( \
	      mkdir -p ../mimalloc/build && \
	      cd ../mimalloc/build && \
	      make \
	  ) && \
	  touch .build; \
	  rm -f .install; \
	fi

install: build
	${Q}if [ ! -e .install ]; then \
	  ( \
	      mkdir -p ../mimalloc/build && \
	      cd ../mimalloc/build && \
	      DESTDIR=${CLEW_3RDPARTY_INSTALL_PATH} make install \
	  ) && \
	  touch .install; \
	fi

clean:
	${Q}${RM} .patch
	${Q}${RM} ../mimalloc/build
	${Q}${RM} .config
	${Q}${RM} .build
	${Q}${RM} .install
