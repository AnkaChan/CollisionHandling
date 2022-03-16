#version 330 core

in vec2 textCoordScreen;
// Ouput data
out vec3 color;
uniform sampler2D reflectedRenderBuffer;

void main()
{
	// Output color = red 
	// color = vec3(1,0,0);
	vec2 uv = ((textCoordScreen.xy + 1.0) / 2.0);
	color = texture( reflectedRenderBuffer, uv ).rgb;
	
}