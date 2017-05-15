
#!/bin/sh
boot=$1

echo ${boot}

boot_path="/media/grouse/"${boot}
echo $boot_path
echo "Copying dtb files to "$boot_path 
cp arch/arm/boot/dts/*var*.dtb $boot_path
