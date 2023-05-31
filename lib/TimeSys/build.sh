#!/bin/bash

# 检查build文件夹是否存在
if [ -d "build" ]; then
    # 提示用户是否删除已有的build文件夹
    read -p "The 'build' directory already exists. Do you want to remove it and create a new one? (y/n): " choice

    # 根据用户的选择执行相应的操作
    if [ "$choice" == "y" ] || [ "$choice" == "Y" ]; then
        rm -rf build
        echo "Removed the existing 'build' directory."
    else
        echo "Aborted. Exiting script."
        exit 0
    fi
fi

mkdir build
cd build || exit
cmake ..
make -j 4
