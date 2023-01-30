# TraceIt

A raytracing sandbox, including an editor for scene setup and configuration, supporting simple objects like spheres, but also more advanced stuff like ray marching in curved spacetime.

I started this project as a basic 3D raytracer with the usual "hello world" stuff, but the ultimate goal was the accurate rendering of some metrics of general relativity (wormholes, black holes), continuing my work from a [previous project]( https://github.com/BjoB/gros). :slightly_smiling_face:

 It is inspired by a lot of fantastic open source content, starting from the well known "Raytracing in one weekend" series [[1]](#1) over to TheCherno's youtube channel on game engines and graphics programming [[2]](#2) to the papers by Kip S. Thorne and the DNEG special effects team on GR raytracing, which were published after the release of the movie "Interstellar" [[3]](#3), [[4]](#4).

# Dependencies

As a prerequisite VulkanSDK-1.3.236.0 (or later) has to be installed.
Tested on Win 10, but should be adaptable to other systems.

Libraries automatically bundled with this repo:

- [ImGui](https://github.com/ocornut/imgui), excellent and  lightweight GUI library
- glm (OpenGL Mathematics, header-only)
- glfw-3.3.8 (header + precompiled lib/dll)
- stb (for image loading)

# References

<a id="1">[1]</a> https://raytracing.github.io/books/RayTracingInOneWeekend.html
<a id="2">[2]</a> https://www.youtube.com/@TheCherno
<a id="3">[3]</a> Kip S. Thorne et al. (2015), [Visualizing Interstellar's Wormhole](https://arxiv.org/abs/1502.03809)
<a id="4">[4]</a> Kip S. Thorne et al. (2015), [Gravitational lensing by spinning black holes in astrophysics, and in the movie Interstellar](https://arxiv.org/abs/1502.03808)
<a id="4">[5]</a> Mykhailo Moroz, https://michaelmoroz.github.io/TracingGeodesics/
<a id="5">[6]</a> https://www.youtube.com/@scottmanley
