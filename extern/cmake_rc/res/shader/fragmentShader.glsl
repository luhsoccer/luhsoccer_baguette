#version 330 core

// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-8-basic-shading/#modeling-the-light

layout(location = 0) out vec4 color;

in vec2 uv; // texture
in vec3 position_worldspace;
in vec3 normal_cameraspace;
in vec3 eye_direction_cameraspace;
in vec3 light_direction_cameraspace;
in vec4 color_out;

uniform vec3 light_pos_worldspace;
uniform sampler2D texture_sampler; // texture

uniform int type; // type = 0 -> textured, type == 1 -> 2d lines/shapes, type == 2 -> colored

void main(){
	// Light emission properties
	// You probably want to put them as uniforms
	vec3 light_color = vec3(1, 1, 1);
	float light_power = 150.0f;
	
    vec3 mat_diffuse_color;
    float alpha = 1;
    if(type == 0){
        // texture
        mat_diffuse_color = texture(texture_sampler, uv).rgb;
    }else{
        // 2d lines/shapes or colored
        mat_diffuse_color = color_out.rgb;
        alpha = color_out.a;
    }

	// Material properties
	vec3 mat_ambient_color = vec3(0.8, 0.8, 0.8) * mat_diffuse_color;
	vec3 mat_specular_color = vec3(0.1, 0.1, 0.1);

	// Distance to the light
	float distance = 15; // length(light_pos_worldspace - position_worldspace);

	// Normal of the computed fragment, in camera space
	vec3 n = normalize(normal_cameraspace);
	// Direction of the light (from the fragment to the light)
	vec3 l = normalize(light_direction_cameraspace);
	// Cosine of the angle between the normal and the light direction, 
	// clamped above 0
	//  - light is at the vertical of the triangle -> 1
	//  - light is perpendiular to the triangle -> 0
	//  - light is behind the triangle -> 0
	float cos_theta = clamp(dot(n, l), 0, 1);
	
	// Eye vector (towards the camera)
	vec3 e = normalize(eye_direction_cameraspace);
	// Direction in which the triangle reflects the light
	vec3 r = reflect(-l, n);
	// Cosine of the angle between the Eye vector and the Reflect vector,
	// clamped to 0
	//  - Looking into the reflection -> 1
	//  - Looking elsewhere -> < 1
	float cos_alpha = clamp(dot(e, r), 0, 1);
	
	color = vec4(
		// Ambient : simulates indirect lightinga
		mat_ambient_color +
		// Diffuse : "color" of the object
		mat_diffuse_color * light_color * light_power * cos_theta / (distance * distance) +
		// Specular : reflective highlight, like a mirror
		mat_specular_color * light_color * light_power * 0 * pow(cos_alpha, 5) / (distance * distance), alpha);
}