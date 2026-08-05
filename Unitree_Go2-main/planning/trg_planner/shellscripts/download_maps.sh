#!/bin/bash

# Download the file
DATA_DIR="../prebuilt_maps"
mkdir -p $DATA_DIR

URL="https://urserver.kaist.ac.kr/publicdata/TRG-planner/prebuilt_map_data.zip"

wget -c $URL -P $DATA_DIR

# Unzip the file
unzip $DATA_DIR/prebuilt_map_data.zip -d $DATA_DIR

# Remove the zip file
rm $DATA_DIR/prebuilt_map_data.zip
