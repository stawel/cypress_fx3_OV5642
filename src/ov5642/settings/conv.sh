#!/bin/bash

sed -e 's/write_i2c(/{/g' -e 's/);/},/g' -e 's/),/},/g' -e 's/^;/\/\//g'  $1