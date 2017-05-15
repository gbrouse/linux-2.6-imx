#!/bin/sh
# execute by typing ". clean.sh"
echo on
# Save the current directory and switch to tools directory
pushd /opt/fsl-imx-x11/4.1.15-1.2.0/
# Echo the new directory
pwd
# Setup the environment
source environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
# Restore the original directory
popd
# Echo the directory
pwd
# Clean up
make mrproper

