#version 330 core

// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-8-basic-shading/#modeling-the-light

layout(location = 0) in vec3 vertex_position_modelspace;
layout(location = 1) in vec3 vertex_normal_modelspace;
layout(location = 2) in vec2 vertex_uv; // texture
layout (location = 3) in vec4 vertex_color; // 2d lines/shapes

out vec4 color_out; // color 
out vec2 uv; // texture

out vec3 position_worldspace;
out vec3 normal_cameraspace;
out vec3 eye_direction_cameraspace;
out vec3 light_direction_cameraspace;

uniform mat4 mvp;
uniform mat4 v;
uniform mat4 m;
uniform mat4 mv;
uniform vec3 light_pos_worldspace;
uniform vec4 color; // color
uniform int multicolor; // 2d lines/shapes

uniform int type; // type = 0 -> textured, type == 1 -> 2d lines/shapes, type == 2 -> colored

void main(){
	// Output position of the vertex, in clip space : MVP * position
	gl_Position = mvp * vec4(vertex_position_modelspace, 1);

	// Position of the vertex, in worldspace : M * position
	position_worldspace = (m * vec4(vertex_position_modelspace, 1)).xyz;

	// Vector that goes from the vertex to the camera, in camera space.
	// In camera space, the camera is at the origin (0,0,0).
	eye_direction_cameraspace = vec3(0,0,0) - (mv * vec4(vertex_position_modelspace, 1)).xyz;

	// Vector that goes from the vertex to the light, in camera space
	vec3 light_pos_cameraspace = ( v * vec4(light_pos_worldspace, 1)).xyz;
	light_direction_cameraspace = light_pos_cameraspace + eye_direction_cameraspace;

    vec3 normal_modelspace = vertex_normal_modelspace;
    if(type == 0){
        // set texture coordinates
        uv = vertex_uv;
    }else if(type == 1){
        // set normal to point upwards
        normal_modelspace = vec3(0, 1, 0);
        if(multicolor == 1){
            // set color per vertex
            color_out = vertex_color;
        }else{
            // set color for all vertices
            color_out = color;
        }
    }else{
        // set color for whole mesh
        color_out = color;
    }

    // Normal of the the vertex, in camera space
	normal_cameraspace = (mv * vec4(vertex_normal_modelspace, 0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.
}