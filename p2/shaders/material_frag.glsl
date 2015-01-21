varying vec3 norm;
varying vec3 cam_dir;
varying vec3 color;
uniform samplerCube cubemap;

// Declare any additional variables here. You may need some uniform variables.

void main(void)
{
	// Your shader code here.
	vec3 reflectedDirection = reflect(cam_dir, normalize(norm));

	gl_FragColor = textureCube(cubemap, reflectedDirection);
}
