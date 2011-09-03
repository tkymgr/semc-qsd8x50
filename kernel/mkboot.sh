#!/bin/bash
rm -f ../build-bootimg/zImage
mv arch/arm/boot/zImage ../build-bootimg/
pushd ./
cd ../build-bootimg/
./makeit.sh
popd

