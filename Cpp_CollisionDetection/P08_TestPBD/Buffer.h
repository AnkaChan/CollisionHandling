#ifndef _BUFFER_H_
#define _BUFFER_H_
#include <GL/glew.h>
#include "cyCodeBase/cyTriMesh.h"

template <typename T>
struct BufferT
{
	BufferT() :
		pBuf_(nullptr),
		size_(0)
	{};

	BufferT(size_t size) :
		pBuf_(new T[size]),
		size_(size)
	{
	}

	void resize(size_t size) {
		delete[] pBuf_;
		pBuf_ = new T[size];
		size_ = size;
	}

	~BufferT() {
		delete[] pBuf_;
	}

	T* getBufHead() {
		return pBuf_;
	}

	size_t getByteSize() {
		return size_ * sizeof(T);
	}

	size_t getSize() {
		return size_;
	}

private:
	T * pBuf_;
	size_t size_;
};

typedef BufferT<GLfloat> BufferF;
typedef BufferT<GLint> BufferI;

void getTriangleArrayBuffer(const float * pData, size_t size, BufferF & buf) {
	buf.resize(size);

	for (size_t i = 0; i < size; i++)
	{
		buf.getBufHead()[i] = pData[i];
	}
}

void getTriangleArrayBuffer(const cyTriMesh& mesh, BufferF & buf) {
	size_t bufSize = 3 * 3 * mesh.NF();
	buf.resize(bufSize);

	for (size_t i = 0; i < mesh.NF(); i++)
	{
		// Iterate over faces
		for (size_t j = 0; j < 3; j++)
		{
			// Iterate over face vertices
			for (size_t k = 0; k < 3; k++)
			{
				// Iterate over vertex coordinates
				buf.getBufHead()[i * 3 * 3 + j * 3 + k] = mesh.V(mesh.F(i).v[j])[k];
			}
		}
	}
}

void getTriangleNormalArrayBuffer(const cyTriMesh& mesh, BufferF & buf) {
	size_t bufSize = 3 * 3 * mesh.NF();
	buf.resize(bufSize);

	for (size_t i = 0; i < mesh.NF(); i++)
	{
		// Iterate over faces
		for (size_t j = 0; j < 3; j++)
		{
			// Iterate over face vertices
			for (size_t k = 0; k < 3; k++)
			{
				// Iterate over vertex coordinates
		
				buf.getBufHead()[i * 3 * 3 + j * 3 + k] = mesh.VN(mesh.FN(i).v[j])[k];
			}
		}
	}

}

void getTriangleUvArrayBuffer(const cyTriMesh& mesh, BufferF& buf) {
	size_t bufSize = 3 * 2 * mesh.NF();
	buf.resize(bufSize);

	for (size_t i = 0; i < mesh.NF(); i++)
	{
		// Iterate over faces
		for (size_t j = 0; j < 3; j++)
		{
			// Iterate over face vertices
			for (size_t k = 0; k < 2; k++)
			{
				// Iterate over vertex coordinates

				buf.getBufHead()[i * 3 * 2 + j * 2 + k] = mesh.VT(mesh.FT(i).v[j])[k];
			}
			// std::cout << "Set uv to:" << buf.getBufHead()[i * 3 * 2 + j * 2 + 0] << " " << buf.getBufHead()[i * 3 * 2 + j * 2 + 1] << '\n';

		}
	}

}

#endif // !_BUFFER_H_
