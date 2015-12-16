#!/bin/sh

#echo $2/include/linux/ 
mkdir -p $2/include/linux
rm -rf $2/include/linux/tablet_config.h
touch $2/include/linux/tablet_config.h

echo "#ifndef __TABLET_CONFIG_H__" >> $2/include/linux/tablet_config.h
echo "#define __TABLET_CONFIG_H__" >> $2/include/linux/tablet_config.h

[[ $1 =~ "YT2_13" ]] && echo "#define BLADE2_13 " >> $2/include/linux/tablet_config.h
[[ $1 =~ "YT2_10" ]] && echo "#define BLADE2_10 " >> $2/include/linux/tablet_config.h
[[ $1 =~ "YT2_8" ]] && echo "#define BLADE2_8 " >> $2/include/linux/tablet_config.h

echo "#endif" >> $2/include/linux/tablet_config.h

