#!/bin/bash
OUTPUT_DIR=$1
nrfutil pkg generate --hw-version 52 --application-version 1 --application ./projects/proto_fw_argos/segger/Output/Release/Exe/proto_fw_argos_v1.hex --sd-req 0xB7 --key-file ./keys/private.key app.zip
cp app.zip $OUTPUT_DIR
