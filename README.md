# nmpc_tracking

## Environment

ROS, Ubuntu 20.04 + ROS

## Install

### 1. Casadi setup
<font color="#696969">*Last edited time: 2024/10/09*</font>

1.安裝必要的編譯器
```bash
$ sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
```
2.下載Casadi程式碼
```bash
$ git clone https://github.com.casadi
```
3.編譯安裝
```bash
$ cd casadi
$ mkdir build
$ cd build
$ cmake .. -DWITH_IPOPT=ON -FWITH_EXAMPLES=OFF
$ make
$ sudo make install
```
> Ref. https://blog.csdn.net/qq_41701758/article/details/131527719

### 2. Ipopt setup
<font color="#696969">*Last edited time: 2024/10/09*</font>

1.安裝ASL
```bash
$ git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
$ cd ThirdParty-ASL
$ ./get.ASL
$ ./configure
$ make
$ sudo make install
```
2.安裝Ipopt
```bash
$ git clone https://github.com/coin-or/Ipopt.git
$ cd Ipopt
$ mkdir build
$ cd build
$ ~/Ipopt/configure
$ make
$ make test #optional
$ sudo make install
```
> Ref. (install ASL as linear solver) https://coin-or.github.io/Ipopt/INSTALL.html

