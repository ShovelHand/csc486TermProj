  #version 330 core
        // uniform vec3 LDIR; ///< TODO: fix me
        in vec3 fnormal;
        in vec3 fbaryc;
        out vec4 fcolor;
    
        // http://codeflow.org/entries/2012/aug/02/easy-wireframe-display-with-barycentric-coordinates
        float edgeFactor(){
            vec3 d = 1.5 * fwidth(fbaryc);
            vec3 a3 = smoothstep(vec3(0.0), d, fbaryc);
            return min(min(a3.x, a3.y), a3.z);
        }
        
        void main(){
            ///--- Face flat shading
            vec3 LDIR = vec3(0,0,1);
            vec3 basecolor = vec3(1,0,0);
            vec3 ldir = normalize(LDIR);
            float albedo = max( dot( normalize(fnormal), ldir ), 0 );   
            vec4 face_color = vec4(basecolor*albedo, 1);
            
            ///--- colors & mix
            vec4 edge_color = vec4(0,0,0,1);
            fcolor = mix(edge_color, face_color, edgeFactor());
        }