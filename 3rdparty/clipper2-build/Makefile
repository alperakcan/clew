
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
	      mkdir -p ../clipper2/CPP/build && \
	      cd ../clipper2/CPP/build && \
	      CFLAGS=-g \
	      LDFLAGS= \
	      MAKEFLAGS= \
	      cmake \
	        -DCLIPPER2_EXAMPLES=OFF \
	        -DCLIPPER2_TESTS=OFF \
	        -DBUILD_SHARED_LIBS=OFF \
		-DCLIPPER2_HI_PRECISION=OFF \
		-DCLIPPER2_UTILS=OFF \
	        .. \
	  ) && \
	  touch .config; \
	  rm -f .build; \
	fi

build: config
	${Q}if [ ! -e .build ]; then \
	  ( \
	      mkdir -p ../clipper2/CPP/build && \
	      cd ../clipper2/CPP/build && \
	      make \
	  ) && \
	  touch .build; \
	  rm -f .install; \
	fi

install: build
	${Q}if [ ! -e .install ]; then \
	  ( \
	      mkdir -p ../clipper2/CPP/build && \
	      cd ../clipper2/CPP/build && \
	      DESTDIR=${CLEW_3RDPARTY_INSTALL_PATH} make install \
	  ) && \
	  touch .install; \
	fi

clean:
	${Q}if [ -e .patch ]; then \
	  ( \
	    true \
	  ) && \
	  rm -rf .patch; \
	fi
	${Q}${RM} ../clipper2/CPP/build
	${Q}${RM} .config
	${Q}${RM} .build
	${Q}${RM} .install
