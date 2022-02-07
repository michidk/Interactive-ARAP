builddir := justfile_directory() + "/build"

build:
    #!/usr/bin/env bash
    mkdir -p {{builddir}}
    cmake -DCMAKE_BUILD_TYPE=Debug -S . -B {{builddir}}
    cd {{builddir}}
    make

run:
    {{builddir}}/src/arap

runb: build run

test:
    {{builddir}}/src/tests

testb: build test

clean:
    git clean -Xdf
