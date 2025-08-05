#!/bin/bash

config-pin P9_17 spi_cs
config-pin P9_18 spi
config-pin P9_21 spi
config-pin P9_22 spi_sclk

gcc 12_spi_test.c -o output_12
