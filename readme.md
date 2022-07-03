## Bounce Softbody

Bounce Softbody is a small 3D softbody physics engine.

## Features

### Common

* Efficient data structures with no use of STL
* Fast memory allocators
* Built-in math library
* Tunable settings used across the entire library
* Included debug draw utility

### Sparse

* Fairly efficient sparse matrix and vector structures
* Full Backward Euler method
* Pre-filtered Pre-conditioned Conjugate Gradient (PPCG) method

### Collision

* Spheres, capsules, triangles, boxes, triangle meshes, and signed distance fields (SDFs)
* Dynamic tree

### Dynamics

* Unconditional simulation stability
* Stretch, spring, mouse, triangle and tetrahedron element force types
* Vertex contact, friction
* Viscosity
* Linear/non-linear time solver
* Ray-casting

### Testbed
	
* OpenGL with GLFW and GLAD
* UI by imgui
* Orbit camera
* Mouse picking
* premake build system

## Dependencies

### Testbed

#### External 

These are the external dependencies for the Testbed example project. If you don't care about Testbed, then you don't need these dependencies. 

* [GLFW](https://www.glfw.org/)
* [GLAD](https://glad.dav1d.de/)
* [imgui](https://github.com/ocornut/imgui)

### Documentation

**Note**: Use the the Testbed for learning how to use Bounce Softbody. The Testbed is a collection of visual tests and examples that can support the development of the library. As you would imagine, this application is not part of the library.

## Building

Bounce Softbody uses [premake](https://premake.github.io/) for generating project files in a platform agnostic manner. [premake](https://premake.github.io/) is available at https://premake.github.io/.

* Put premake into bounce_softbody/.

### Windows 

#### Visual Studio 2019

* Say { premake5 vs2019 } on a command line. 
* Open { build/vs2019/bounce_softbody.sln }.
* Set testbed as the startup project.
* In the testbed debugging properties, set Working Directory to '..\\..\testbed'.
* Press F5 to run.

### Linux

From the official GLFW documentation:

"... To compile GLFW for X11, you need to have the X11 packages installed, as well as the basic development tools like GCC and make. For example, on Ubuntu and other distributions based on Debian GNU/Linux, you need to install the xorg-dev package, which pulls in all X.org header packages. ..."

#### GNU Make

##### x86

* Say { ./premake5 gmake2 } on a terminal.
* From build/gmake2 say { make config="debug_x86" }.
* Set the testbed directory as the working directory.
* From bin/x86/debug/testbed say { ./testbed }.

##### x64

* Say { ./premake5 gmake2 } on a terminal.
* From build/gmake2 say { make config="debug_x86_64" }.
* Set the testbed directory as the working directory.
* From bin/x86_64/debug/testbed say { ./testbed }.
