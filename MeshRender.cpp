#include "MeshRender.h"


class MeshRender{

	void MeshRender::init()
	{
		///--- Shader
		program.add_vshader_from_source(vshader);
		program.add_fshader_from_source(fshader);
		program.link();

		///--- Data
		init_data();

		///--- Attributes
		program.bind();
		vao.bind();
		program.set_attribute("vposition", vertexbuffer);
		program.set_attribute("vnormal", normalbuffer);
		program.set_attribute("vbaryc", barycbuffer);
		vao.release();
		program.release();
	}

	void MeshRender::display(){
		program.bind();
		vao.bind();
		glDrawArrays(GL_TRIANGLES, 0, mesh.n_faces() * 3 /*#verts*/);
		vao.release();
		program.release();
	}

	void MeshRender::setSmooth()
	{
		smoothRender = !smoothRender;
		if (smoothRender)
		{
			program.add_vshader_from_source(vshader);
			program.add_fshader_from_source(fshader);
		}
		else
		{
			program.add_vshader_from_source(vshader);
			program.add_fshader_from_source(fshader);
		}
	}

};