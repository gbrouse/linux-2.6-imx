#!/bin/sh
boot=$1
root=$2

echo ${boot}
echo ${root}

boot_path="/media/grouse/"${boot}
root_path="/media//grouse/"${root}
echo $boot_path
echo $root_path
echo "Copying uImage to "$boot_path 
cp arch/arm/boot/uImage  $boot_path
echo "Installing modules to "$root_path 
make ARCH=arm modules_install INSTALL_MOD_PATH=$root_path
echo "Copying dtb files to "$boot_path 
cp arch/arm/boot/dts/*var*.dtb $boot_path
