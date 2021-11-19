# SV-SLAM

# Requirements

## CMake

version: `3.12+`

Normally if your system is `Ubuntu 18.04`, you should upgrade your cmake.

**Instruction：** https://zhuanlan.zhihu.com/p/93480024

## PCL

https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html

Download release version of PCL from https://github.com/PointCloudLibrary/pcl/releases

```
cd pcl && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```

## mmsegmentation

1. Clone packages

   ```bash
   cd lib
   git clone https://github.com/open-mmlab/mmsegmentation.git
   ```

2. `MMSegmentation` Requirements, please refer to <https://mmsegmentation.readthedocs.io/en/latest/get_started.html#installation>

   every time you change the path of `mmsegmentation` must not forget to run

   ```bash
   cd mmsegmentation
   pip install -e .
   ```

## pybind11

1. Clone packages

   ```bash
   cd lib
   git clone https://github.com/pybind/pybind11.git
   ```


## Potential problems

**undefined reference to `TIFFReadRGBAStrip@LIBTIFF_4.0'**

solution： https://blog.csdn.net/wphkadn/article/details/102504573

move/delete `libtiff.so``libtiff.so.5``libtiff.so.5.6.0` under your anaconda env