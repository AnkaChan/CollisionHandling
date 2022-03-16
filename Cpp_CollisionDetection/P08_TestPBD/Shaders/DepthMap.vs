#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

uniform mat4 DepthMapMVP;

void main(){
  // Output position of the vertex, in clip space : MVP * position
  gl_Position =  DepthMapMVP * vec4(vertexPosition_modelspace,1);
  
 }