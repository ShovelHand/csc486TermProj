 #version 330 core
        uniform mat4 M;    
        uniform mat4 MV; 
        uniform mat4 MVP; 
        in vec3 vbaryc;      ///< per-vertex barycentric
        in vec3 vposition;   ///< per-vertex position
        in vec3 vnormal;     ///< per-vertex normal
        out vec3 fnormal;    ///< per-fragment normal
        out vec3 fbaryc;     ///< per-fragment barycentric
        void main(){ 
            gl_Position = MVP * vec4(vposition, 1.0); 
            fnormal = normalize( inverse(transpose(mat3(MV))) * vnormal );
            fbaryc = vbaryc;
        }