#!/bin/bash

echo "Cleaning up previous build..."
rm -rf build

echo "Creating new build directory..."
mkdir build

cd build

echo "Running cmake ..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "Running make ..."
make -j 48

echo "============================================================"
echo "████████╗██████╗  ██████╗                                   "
echo "╚══██╔══╝██╔══██╗██╔════╝                                   "
echo "   ██║   ██████╔╝██║  ███╗█████╗                            "
echo "   ██║   ██╔══██╗██║   ██║╚════╝                            "
echo "   ██║   ██║  ██║╚██████╔╝                                  "
echo "   ╚═╝   ╚═╝  ╚═╝ ╚═════╝                                   "
echo "                                                            "
echo "██████╗ ██╗      █████╗ ███╗   ██╗███╗   ██╗███████╗██████╗ "
echo "██╔══██╗██║     ██╔══██╗████╗  ██║████╗  ██║██╔════╝██╔══██╗"
echo "██████╔╝██║     ███████║██╔██╗ ██║██╔██╗ ██║█████╗  ██████╔╝"
echo "██╔═══╝ ██║     ██╔══██║██║╚██╗██║██║╚██╗██║██╔══╝  ██╔══██╗"
echo "██║     ███████╗██║  ██║██║ ╚████║██║ ╚████║███████╗██║  ██║"
echo "╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝"
echo "============================================================"
echo "Build complete!                                             "
echo "You can now run the project with the following command:     "
echo -e "$ \033[0;32m./build/run_trg_planner mountain \033[0m     "
echo "Change 'mountain' to the name of the map you want to run.   "
echo "============================================================"
