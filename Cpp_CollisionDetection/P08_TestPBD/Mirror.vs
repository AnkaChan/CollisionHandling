#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;
uniform mat4 V;
uniform mat4 P;
uniform mat4 MVP;

//out vec3 Position_worldspace;
//out vec2 textCoordScreen;

void main(){
  // Output position of the vertex, in clip space : MVP * position
  gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
  
}