// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
//using namespace glm;

#include <array>

#define CY_GL_REGISTER_DEBUG_CALLBACK
#include "cyCodeBase/cyGL.h"
#include "cyCodeBase/cyTriMesh.h"
#include "Buffer.h"
#include "GLArray.h"

#include "lodepng.h"

#include "Point.h"

#include "SoftBody.h"

using MeshLib::CPoint;

cyGLSLProgram programRender;
cyGLSLProgram programDepthMap;
cyGLSLProgram programBufferToScreen;

const char* inputVertexShader = "BlinnShading.vertexshader";
const char* inputFragmetsShader = "BlinnShading.fragmentshader";

//const char* inputFragmetsShader = "SimpleFragmentShader.fragmentshader";
//const char* inputFragmetsShader = "VisNormal.fragmentshader";

const int width = 1024;
const int height = 768;

float alpha = 0;
float alphaChange = 0;
float beta = 0;
float betaChange = 0;

float alphaLight = 0;
float alphaChangeLight = 0;
float betaLight = 0;
float betaChangeLight = 0;

float verticalFieldOfView = 45;

float mirrorYPlane[] = {
	-20.0f, -20.0f ,  -5.0f,
	 20.0f, -20.0f ,  -5.0f,
	-20.0f,  20.0f ,  -5.0f,
	-20.0f,  20.0f ,  -5.0f,
	 20.0f, -20.0f ,  -5.0f,
	 20.0f,  20.0f ,  -5.0f,
};

float mirrorYPlaneN[] = {
	0.0f, 0.0f ,  1.0f,
	0.0f, 0.0f ,  1.0f,
	0.0f, 0.0f ,  1.0f,
	0.0f, 0.0f ,  1.0f,
	0.0f, 0.0f ,  1.0f,
	0.0f, 0.0f ,  1.0f,
};

const GLfloat g_quad_vertex_buffer_data[] = {
	-25.0f, -25.0f, 0.0f,
	 25.0f, -25.0f, 0.0f,
	-25.0f,  25.0f, 0.0f,
	-25.0f,  25.0f, 0.0f,
	 25.0f, -25.0f, 0.0f,
	 25.0f,  25.0f, 0.0f,
};

bool altkey = false;

int gButton = 10; //10 correspondes to no button
double startPointX, startPointY;

void keyCallBack(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_F6 && action == GLFW_RELEASE)
	{
		std::cout << "Recompile shaders.\n";
		programRender.BuildFiles("BlinnShadingShadowMapping.vertexshader", "BlinnShadingShadowMapping.fragmentshader");
		programBufferToScreen.BuildFiles("Passthrough.vertexshader", "DirectTexture.fragmentshader");
		programDepthMap.BuildFiles("BlinnShading.vertexshader", "BlinnShading.fragmentshader");
	}	

	if (key == GLFW_KEY_LEFT_ALT && action == GLFW_PRESS)
	{
		std::cout << "Alt pressed.\n";
		altkey = true;
	}
	else if (key == GLFW_KEY_LEFT_ALT && action == GLFW_RELEASE)
	{
		std::cout << "Alt released.\n";

		altkey = false;
	}

	if (key == GLFW_KEY_MINUS && action == GLFW_PRESS) {
		verticalFieldOfView -= 0.5;
	}

	if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS) {
		verticalFieldOfView += 0.5;
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	/* set up an arcball around the Eye's center
	switch y coordinates to right handed system  */
	if (altkey) {
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
			glfwGetCursorPos(window, &startPointX, &startPointY);
			gButton = button;

			alphaChangeLight = 0;
			betaChangeLight = 0;
		}

		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			gButton = 10;

			alphaLight += alphaChangeLight;
			betaLight += betaChangeLight;

			alphaChangeLight = 0;
			betaChangeLight = 0;
		}
	}
	else
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
			glfwGetCursorPos(window, &startPointX, &startPointY);
			gButton = button;

			alphaChange = 0;
			betaChange = 0;
		}

		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			gButton = 10;

			alpha += alphaChange;
			beta += betaChange;

			alphaChange = 0;
			betaChange = 0;
		}
	}

	return;
}

void mouseMoveCallBack(GLFWwindow* window, double xpos, double ypos)
{
	/* rotation, call arcball */
	if (altkey) {
		if (gButton == GLFW_MOUSE_BUTTON_LEFT)
		{
			alphaChangeLight = 0.01 * (xpos - startPointX);
			betaChangeLight = 0.01 * (ypos - startPointY);
		}
	}
	else
	{
		if (gButton == GLFW_MOUSE_BUTTON_LEFT)
		{
			alphaChange = 0.01 * (xpos - startPointX);
			betaChange = 0.01 * (ypos - startPointY);
		}
	}
}

unsigned int loadCubemap(std::vector<std::string> faces, std::vector<std::vector<unsigned char>>& image)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	image.resize(6);

	int width, height, nrComponents;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned textureWidth;
		unsigned textureHeight;
		unsigned error = lodepng::decode(image[i], textureWidth, textureHeight, faces[i].c_str(), LCT_RGB);

		// If there's an error, display it.
		if (error != 0) {
			std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
			return -1;
		}

		unsigned char*data =  image[i].data();

		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}

void genDepthTexture(GLuint& FramebufferName, GLuint& depthTexture) {

	// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
	
	glGenFramebuffers(1, &FramebufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
	// Depth texture. Slower than a depth buffer, but you can sample it later in your shader
	
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, 1024, 1024, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

	// No color output in the bound framebuffer, only depth.
	glDrawBuffer(GL_NONE);
}

GLuint readTexture(std::string file, unsigned& textureWidth, unsigned& textureHeight, std::vector<unsigned char>& image) {
	unsigned error = lodepng::decode(image, textureWidth, textureHeight, file, LCT_RGB);

	// If there's an error, display it.
	if (error != 0) {
		std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
		return 1;
	}
	// Create one OpenGL texture
	GLuint texture;
	glGenTextures(1, &texture);
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, texture);

	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data());
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //GL_NEAREST = no smoothing
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	return texture;
}

int main(int argc, char ** argv)
{

    initPhysics();
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(width, height, "Project 07 - Shadow Mapping", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);
	// Cull triangles which normal is not towards the camera
	glEnable(GL_TEXTURE_2D);

	// Create and compile our GLSL programRender from the shaders
	programRender.BuildFiles("BlinnShadingShadowMapping.vertexshader", "BlinnShadingShadowMapping.fragmentshader");
	programBufferToScreen.BuildFiles("Passthrough.vertexshader", "DirectTexture.fragmentshader");
	programDepthMap.BuildFiles("DepthMap.vs", "DepthMap.fs");
	// Mesh VAO
	cyTriMesh mesh;
	mesh.LoadFromFileObj("Teapot.obj");

	GL_Array meshArr;
	BufferF tvBuf;
	getTriangleArrayBuffer(mesh, tvBuf);
	meshArr.bindVertices(tvBuf.getBufHead(), tvBuf.getByteSize());
	BufferF tnBuf;
	getTriangleNormalArrayBuffer(mesh, tnBuf);
	meshArr.bindNormals(tnBuf.getBufHead(), tnBuf.getByteSize());
	BufferF ttBuf;
	getTriangleUvArrayBuffer(mesh, ttBuf);
	meshArr.bindUVs(ttBuf.getBufHead(), ttBuf.getByteSize());

	glfwSetKeyCallback(window, keyCallBack);
	glfwSetCursorPosCallback(window, mouseMoveCallBack);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	GL_Array texturePlaneArr;
	BufferF texturePlaneVBuf;
	getTriangleArrayBuffer(g_quad_vertex_buffer_data, 18, texturePlaneVBuf);
	texturePlaneArr.bindVertices(texturePlaneVBuf.getBufHead(), texturePlaneVBuf.getByteSize());

	GL_Array yPlaneArr;
	BufferF yPBuf;
	getTriangleArrayBuffer(mirrorYPlane, 18, yPBuf);
	yPlaneArr.bindVertices(yPBuf.getBufHead(), yPBuf.getByteSize());
	BufferF yPNBuf;
	getTriangleArrayBuffer(mirrorYPlaneN, 18, yPNBuf);
	yPlaneArr.bindNormals(yPNBuf.getBufHead(), yPNBuf.getByteSize());
	
	glm::vec3 LightPosition_worldspace(0, 50, 0);
	glm::vec3 CameraPosition_worldspace(0, 50, 0);

	float cameDis = 50;
	float lightDis = 50;

	GLuint dTestFramebuffer, depthBuffer;
	genDepthTexture(dTestFramebuffer, depthBuffer);

	do {
		/****************************************************************************************************/
		// Prepare the transformations
		glm::mat4 Projection = glm::perspective(glm::radians(verticalFieldOfView), (float)width / (float)height, 0.1f, 100.0f);

		glm::mat4 ModelRender = glm::mat4(1.0f);
		float alphaFinal = alpha + alphaChange;
		float betaFinal = beta + betaChange;

		//glm::mat4 invRot2Render = glm::rotate(glm::mat4(1.0f), betaFinal, glm::vec3(1, 0, 0));
		//glm::vec4 axisRot1Render = invRot2Render * glm::vec4(0, 0, 1, 1);
		//ModelRender = glm::rotate(ModelRender, -alphaFinal, glm::vec3(axisRot1Render));
		//ModelRender = glm::rotate(ModelRender, betaFinal, glm::vec3(1, 0, 0));


		float alphaFinalLight = alphaLight + alphaChangeLight;
		float betaFinalLight = betaLight + betaChangeLight;
		LightPosition_worldspace = glm::vec3(lightDis * cos(alphaFinalLight), lightDis * sin(alphaFinalLight) * cos(betaFinalLight), lightDis * sin(alphaFinalLight) * sin(betaFinalLight));
		CameraPosition_worldspace = glm::vec3(cameDis * cos(alphaFinal), cameDis * sin(alphaFinal) * cos(betaFinal), cameDis * sin(alphaFinal) * sin(betaFinal));
		glm::mat4 View = glm::lookAt(
			CameraPosition_worldspace, // Camera is at (0, 50, 50), in World Space
			glm::vec3(0, 0, 0), // and looks at the origin
			glm::vec3(0, 0, 1)  // Head is up (set to 0,-1,0 to look upside-down)
		);

		glm::mat4 mvpRender = Projection * View * ModelRender; // Remember, matrix multiplication is the other way around

		/****************************************************************************************************/
		// Prepare the depth buffer from light perspective
		glBindFramebuffer(GL_FRAMEBUFFER, dTestFramebuffer);
		glViewport(0, 0, 1024, 1024); // Render on the whole framebuffer, complete from the lower left corner to the upper right

		//glEnable(GL_CULL_FACE);
		//glCullFace(GL_BACK); // Cull back-facing triangles -> draw only front-facing triangles

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programDepthMap.GetID());
		meshArr.avtivate();

		glm::vec3 lightInvDir = glm::vec3(0.5f, 2, 2);

		// Compute the MVP matrix from the light's point of view
		//glm::mat4 depthProjectionMatrix = glm::ortho<float>(-10, 10, -10, 10, -10, 20);
		//glm::mat4 depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
		// or, for spot light :
		//glm::vec3 lightPos(5, 20, 20);
		glm::mat4 depthProjectionMatrix = glm::perspective<float>(glm::radians(45.0f), 1.0f, 20.0f, 80.0f);
		glm::mat4 depthViewMatrix = glm::lookAt(LightPosition_worldspace, glm::vec3(0, 0, 0), glm::vec3(0,0,1));

		glm::mat4 depthModelMatrix = ModelRender;
		glm::mat4 depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;

		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		programDepthMap.SetUniformMatrix4("DepthMapMVP", &depthMVP[0][0]);

		// Draw the triangles !
		glDrawArrays(GL_TRIANGLES, 0, tvBuf.getSize());
		meshArr.deactivate();

		/****************************************************************************************************/
		// Visualize the Depth buffer
		//glDisable(GL_CULL_FACE);
		//glClearColor(0.4f, 0.4f, 0.4f, 0.0f);
		//glBindFramebuffer(GL_FRAMEBUFFER, 0);
		//glViewport(0, 0, width, height);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//// Use our shader
		//glUseProgram(programBufferToScreen.GetID());
		//programBufferToScreen.SetUniformMatrix4("MVP", &mvpRender[0][0]);
		//programBufferToScreen.SetUniform("renderedTexture", 0);
		//// Bind our texture in Texture Unit 0
		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, depthBuffer);
		//// Set our "renderedTexture" sampler to use Texture Unit 0
		//texturePlaneArr.avtivate();
		//glDrawArrays(GL_TRIANGLES, 0, 6);
		//texturePlaneArr.deactivate();

		/****************************************************************************************************/
		//// Render to Screen
		// glEnable(GL_CULL_FACE);

		glClearColor(0.4f, 0.4f, 0.4f, 0.0f);

		// Render to our framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 worldMatrix(1.0f);

		glm::mat4 biasMatrix(
			0.5, 0.0, 0.0, 0.0, 
			0.0, 0.5, 0.0, 0.0,
			0.0, 0.0, 0.5, 0.0,
			0.5, 0.5, 0.5, 1.0
		);

		glm::mat4 depthBiasMVP = biasMatrix*depthMVP;

		// Use our shader
		glUseProgram(programRender.GetID());
		programRender.SetUniformMatrix4("MVP", &mvpRender[0][0]);
		programRender.SetUniformMatrix4("DepthMapMVP", &depthBiasMVP[0][0]);
		programRender.SetUniformMatrix4("V", &View[0][0]);
		programRender.SetUniformMatrix4("M", &ModelRender[0][0]);
		programRender.SetUniform("LightPosition_worldspace", LightPosition_worldspace.x, LightPosition_worldspace.y, LightPosition_worldspace.z);
		programRender.SetUniform("shadowMap", 0);

		//// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, depthBuffer);

		meshArr.avtivate();
		glDrawArrays(GL_TRIANGLES, 0, tvBuf.getSize());
		meshArr.deactivate();

		yPlaneArr.avtivate();
		glDrawArrays(GL_TRIANGLES, 0, yPBuf.getSize());
		yPlaneArr.deactivate();

		/****************************************************************************************************/
		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Cleanup VBO
	glDeleteProgram(programRender.GetID());

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}




