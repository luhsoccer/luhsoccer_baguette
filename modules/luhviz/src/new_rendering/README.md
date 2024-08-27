# New Rendering
For the rendering of the 3D models and the field OpenGL 4.5 is used.
To have a clear defined lifetime of buffers in OpenGL and to organize the code better, there was created a wrapper class for the following components:
- gl_buffer
- gl_contex
- gl_shader_program
- gl_shader
- gl_texture
- gl_uniform
- gl_vertex_array

How OpenGL works will not be explained in detail here because that would be out of scope. Each of the classes use the constructor and destructor to allocate memory and respective free it. For example the gl_buffer generates the buffer in the constructor and deletes the buffer in the destructor. The gl_context defines simple wrapper function to draw vertex arrays, swap current shader program and so on.
This classes therefore form a solide base to be used in the [RenderView](../render_view/README.md) component.