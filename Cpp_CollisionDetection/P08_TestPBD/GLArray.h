#ifndef _GL_ARRAY_H_
#define _GL_ARRAY_H_
#include <GL/glew.h>

class GL_Array
{
public:
	GL_Array() {
		glGenVertexArrays(1, &arrayId_);
	}
	~GL_Array() {
		glDeleteVertexArrays(1, &arrayId_);

	}

	void bindVertices(GLfloat* pData, size_t size) {
		glBindVertexArray(arrayId_);
		glGenBuffers(1, &vBufId_);
		glBindBuffer(GL_ARRAY_BUFFER, vBufId_);
		glBufferData(GL_ARRAY_BUFFER, size, pData, GL_STATIC_DRAW);
		hasVertices_ = true;
	}

	void bindNormals(GLfloat* pData, size_t size) {
		glBindVertexArray(arrayId_);
		glGenBuffers(1, &nBufId_);
		glBindBuffer(GL_ARRAY_BUFFER, nBufId_);
		glBufferData(GL_ARRAY_BUFFER, size, pData, GL_STATIC_DRAW);
		hasNormals_ = true;
	}

	void bindUVs(GLfloat* pData, size_t size) {
		glBindVertexArray(arrayId_);
		glGenBuffers(1, &uvBufId_);
		glBindBuffer(GL_ARRAY_BUFFER, uvBufId_);
		glBufferData(GL_ARRAY_BUFFER, size, pData, GL_STATIC_DRAW);
		hasUVs_ = true;
	}

	void avtivate() {
		if (hasVertices_)
		{
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vBufId_);
			glVertexAttribPointer(
				0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*)0            // array buffer offset
			);
		}
		// 2rst attribute buffer : normals
		if (hasNormals_)
		{
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, nBufId_);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
		}
		// 3rst attribute buffer : uvs
		if (hasUVs_)
		{
			glEnableVertexAttribArray(2);
			glBindBuffer(GL_ARRAY_BUFFER, uvBufId_);
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
		}
	}

	void deactivate() {
		if (hasVertices_)
		{
			glDisableVertexAttribArray(0);
		}
		if (hasNormals_)
		{
			glDisableVertexAttribArray(1);
		}
		if (hasUVs_)
		{
			glDisableVertexAttribArray(2);
		}
	}

private:
	GLuint arrayId_;
	GLuint vBufId_;
	GLuint nBufId_;
	GLuint uvBufId_;

	bool hasVertices_ = false;
	bool hasNormals_ = false;
	bool hasUVs_ = false;
};



#endif // !_GL_ARRAY_H_
