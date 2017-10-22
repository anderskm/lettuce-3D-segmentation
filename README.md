# Segmentation of Lettuce in Coloured 3D Point Clouds for Fresh Weight Estimation
This repository contains the source code for the novel segmentation method of lettuce in coloured 3D point clouds presented in journal paper [Segmentation of Lettuce in Coloured 3D Point Clouds for Fresh Weight Estimation](https://github.com/anderskm/lettuce-3D-segmentation).

The [`src`](src/) folder contains the source code along with Makefiles and program descriptions.

The [`data`](data/) folder contains examples of coloured 3D point clouds of Cos and Iceberg lettuce.

## Citation
If you use this code in your research or elsewhere, please cite/reference the following paper:
[Segmentation of Lettuce in Coloured 3D Point Clouds for Fresh Weight Estimation](https://github.com/anderskm/lettuce-3D-segmentation)
```
@article{Mortensen2017,
  title={Segmentation of Lettuce in Coloured 3D Point Clouds for Fresh Weight Estimation},
  author={Mortensen, Anders Krogh and Bender, Asher and Whelan, Brett and Barbour, Margaret and Sukkarieh, Salah and Karstoft, Henrik and Gislum, René},
  journal={TBD},
  year={2017}
}
```

## Requirements
The programs are written in C++ using the [Point Cloud Library (PCL) v1.7](http://pointclouds.org/), [Boost](http://www.boost.org/), [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) and [VTK](https://www.vtk.org/).
The former three are mandatory for PCL.

The programs were developed in [Qt Creator (v5.7)](https://www.qt.io/) and using [CMake](https://cmake.org/) to building the programs. `CMakeFiles` for all programs are included.

## Installation

1. Download or clone this repository
2. For each program in the `src` folder, create a new project in Qt Creator and build the program:
    1. When creating a new project, the following settings were used:
        * Project type: "Non-Qt-Project" and "Plant C++ Application"
        * Build system: "CMake"
        * Kit selection: "Desktop Qt 5.7.0 GCC 64bit"
        * Project Management: `<None>`
    1. Copy the CMake-file and source code from the corresponding program to the new project folder.
        * Overwrite the existing CMake-file in the project folder.
    1. Setup the build folder by selecting the "Project" icon in the left pane. Changed the "Build directory" to what suits you. The program will be stored in this folder.
    1. In Qt Creator, reconfigure CMake by selecting "Build" > "Run CMake".
    1. Build the project by selecting "Build" > "Build Project 'Project name'" or press `Ctrl` + `B`.
    1. (Optional) Copy the build program into a `PATH`-folder, so it can be accessed from any folder. 

## Usage
See descriptions of the individual programs located in the [`src`](src/) folder.