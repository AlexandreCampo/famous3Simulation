#!/usr/bin/env bash

dirs=`find . -maxdepth 2 -name premake4.lua -exec dirname {} \;`

echo $dirs

for i in $dirs
do
    echo ======================================================
    echo ======== Compiling $i =========
    echo ======================================================
       
    cd $i
    premake4 gmake
    make clean
    make
    cd ..
done
