#!/bin/sh
# execute by typing ". build-all.sh"
echo on
# Save the current directory and switch to tools directory
pushd /opt/fsl-imx-x11/4.1.15-1.2.0/
# Echo the new directory
pwd
# Setup the environment
source environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
# Restore the original directory
popd
# Echo the irectory
pwd
# Clean up
make mrproper
# Select the configuration
make imx_v7_var_defconfig
# Build the kernel image
make -j4 LOADADDR=0x10008000 uImage
# Build the linux modules
make -j4 LOADADDR=0x10008000 modules
# Build the device tree files
make -j4 dtbs
