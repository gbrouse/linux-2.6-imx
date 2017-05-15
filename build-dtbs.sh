
#!/bin/sh
# execute by typing ". build-dtbs.sh"
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
#make mrproper
# Select the configuration
make imx_v7_var_defconfig
# Build the device tree files
make -j4 dtbs
