# TraceIt - GR raytracer

A raytracing sandbox, including an editor for scene setup and configuration, which supports 3D raytracing of simple objects like spheres and ray marching in curved spacetime, based on the laws of general relativity.

I started this project as a basic 3D raytracer with the usual "hello world" stuff, but the ultimate goal was the accurate rendering of some metrics of general relativity (wormholes, black holes), continuing my work from a [previous project]( https://github.com/BjoB/gros). :slightly_smiling_face:

 It is inspired by a lot of fantastic open source content, starting from the well known "Raytracing in one weekend" series [[1]](#1) over to TheCherno's youtube channel on game engines and graphics programming [[2]](#2) to the papers by Kip S. Thorne et. al. on GR raytracing, which were published after the release of the movie "Interstellar" [[3]](#3), [[4]](#4).

# Wormhole Renderer

Also part of this project is a standalone wormhole renderer in form of a CLI application. It allows you to easily create your own realistic image sequences and videos. It is based on the modified Ellis wormhole metric from the mentioned paper, which was also used by the DNEG special effects team in the movie "Interstellar". You can pass the following arguments, allowing you to set up your own scene, including image size, wormhole parameters, camera position and velocities. Several celestial background sphere textures can be chosen to set the "background" of the two connected space time regions.

As an example, the following call would create a 5s sequence of 640x480 images @30 fps, starting from a given distance and rotation around the wormhole.
Two celestial sphere textures are taken from https://www.dneg.com/, which were provided for educational purposes. In general the id can be set to a value between 0..7 for the available textures.

```bash
traceit_cli.exe 
-w 1080 -h 720 
--distance 10.0 
--radial_velo 0.0 
--azimuth_velo 0.4 
--duration 5.0 
--lower_sphere_id 0  # "Saturn side"
--upper_sphere_id 2  # "Far Galaxy side"
```

Example output video, which can be generated from the image sequence with programs like ffmpeg:

https://user-images.githubusercontent.com/29594555/225776423-2ecfcc52-237c-4860-81b5-d9cac418349d.mp4

# Dependencies

As a prerequisite VulkanSDK-1.3.236.0 (or later) has to be installed, which is used for the GUI application.
Tested on Win 10, but should be adaptable to other systems. Developed with Visual Studio 2022.

Libraries automatically bundled with this repo:

- [ImGui](https://github.com/ocornut/imgui), excellent and  lightweight GUI library
- glm (OpenGL Mathematics, header-only)
- glfw-3.3.8 (header + precompiled lib/dll)
- stb (for image loading)

# Todos

- Add more things to the 3D setup (objects, materials ...)
- Support for black hole metrics

# References

<a id="1">[1]</a> https://raytracing.github.io/books/RayTracingInOneWeekend.html

<a id="2">[2]</a> https://www.youtube.com/@TheCherno

<a id="3">[3]</a> Kip S. Thorne et al. (2015), [Visualizing Interstellar's Wormhole](https://arxiv.org/abs/1502.03809)

<a id="4">[4]</a> Kip S. Thorne et al. (2015), [Gravitational lensing by spinning black holes in astrophysics, and in the movie Interstellar](https://arxiv.org/abs/1502.03808)

<a id="4">[5]</a> Mykhailo Moroz, https://michaelmoroz.github.io/TracingGeodesics/
