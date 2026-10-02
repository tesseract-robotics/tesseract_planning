set -e

# ln -s $BUILD_PREFIX/bin/x86_64-conda-linux-gnu-gcc $BUILD_PREFIX/bin/gcc

if [[ "$OSTYPE" == "darwin"* ]]; then
    TCMALLOC_LIB_PATH="$PREFIX/lib/libtcmalloc_minimal.dylib"
    # conda's clang adds -fvisibility-inlines-hidden, hiding Cereal's visibility("default")
    # registration singleton; with Mach-O's two-level namespace each dylib gets its own registry
    # and consumers throw "unregistered polymorphic type". Strip it so the registry is shared.
    export CXXFLAGS="${CXXFLAGS//-fvisibility-inlines-hidden/}"
else
    TCMALLOC_LIB_PATH="$PREFIX/lib/libtcmalloc_minimal.so"
fi

colcon build --merge-install --install-base="$PREFIX/opt/tesseract_robotics" \
   --event-handlers console_direct+  \
   --packages-ignore gtest osqp osqp_eigen piqp tesseract_examples vhacd \
   --cmake-args -GNinja \
   -DCMAKE_BUILD_TYPE=Release \
   -DBUILD_SHARED_LIBS=ON \
   -DBUILD_IPOPT=OFF \
   -DBUILD_SNOPT=OFF \
   -DCMAKE_PREFIX_PATH:PATH="$PREFIX" \
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF \
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF \
   -DTESSERACT_ENABLE_EXAMPLES=OFF \
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=ON \
   -DSETUPTOOLS_DEB_LAYOUT=OFF \
   -DTESSERACT_ENABLE_TESTING=ON \
   -DTRAJOPT_ENABLE_TESTING=OFF \
   -DTRAJOPT_ENABLE_BENCHMARKING=OFF \
   -DTRAJOPT_ENABLE_RUN_BENCHMARKING=OFF \
   -DTESSERACT_WARNINGS_AS_ERRORS=OFF \
   -DTRAJOPT_WARNINGS_AS_ERRORS=OFF \
   -DCMAKE_VERBOSE_MAKEFILE=ON \
   -Dtcmalloc_minimal_LIBRARY=$TCMALLOC_LIB_PATH

export TESSERACT_RESOURCE_PATH="$PREFIX/opt/tesseract_robotics/share"

colcon test --event-handlers console_direct+  \
   --return-code-on-test-failure \
   --packages-ignore osqp osqp_eigen vhacd \
   --packages-select tesseract_planning \
   --merge-install --install-base="$PREFIX/opt/tesseract_robotics"  


for CHANGE in "activate" "deactivate"
do
    mkdir -p "${PREFIX}/etc/conda/${CHANGE}.d"
    cp "${RECIPE_DIR}/${CHANGE}.sh" "${PREFIX}/etc/conda/${CHANGE}.d/${PKG_NAME}_${CHANGE}.sh"
done