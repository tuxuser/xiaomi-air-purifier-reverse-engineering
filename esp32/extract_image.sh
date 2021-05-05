#!/bin/sh

mkdir partitions.out

for part_name in nvs otadata phy_init miio_fw1 miio_fw2 test mimcu coredump minvs
do
  python esp32_image_parser.py dump_partition $1 -partition $part_name -output partitions.out/$part_name.bin
done
