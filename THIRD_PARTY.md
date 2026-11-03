# Third-party material

CYAN-authored source is covered by the repository MIT license. The following
separable material retains its upstream license.

## HaterMIPS

The R4000-class CPU at `rtl/cpu` is maintained as a separate repository and is
pinned by CYAN. HaterMIPS is MIT licensed. Its optional CHERI/BERI and Berkeley
TestFloat adapters require separately obtained upstream projects; those
projects are not redistributed by CYAN or HaterMIPS.

## Xilinx generated IP

No generated Xilinx IP source or object product is distributed. The KCU105
flow asks an installed, separately licensed Vivado toolchain to generate the
required IP during a build. Generated output remains outside Git.

## Excluded material

This repository contains no SGI firmware, operating-system image, original
vendor manual, proprietary diagnostic binary, or FPGA bitstream.
