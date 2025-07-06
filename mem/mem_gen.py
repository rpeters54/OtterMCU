#!/usr/bin/env python3

import os
import sys

size = int(sys.argv[1])

if size > 32 or size <= 0:
    raise IOError(f"Memory size 2**{size} exceeds 2**1 to 2**32 bounds")

with open('memory.mem', 'w') as f:
    num_bytes = 2**size
    for i in range(num_bytes):
        f.write(f"{(i%256):02X}\n")
