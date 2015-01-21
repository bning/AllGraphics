varying vec3 normal;
varying vec3 cam_dir;
varying vec3 color;
uniform samplerCube cubemap;

void main(void)
{
    // Light Spot factor
    vec3 spotFactor = normalize(normalize(normal) * normalize(cam_dir));

    // Reflection factor
    vec3 reflectedDirection = reflect(cam_dir, normalize(normal));

    // Mix all the factors together
    vec3 mixture = 2.9 * spotFactor + 0.1 * textureCube(cubemap, reflectedDirection);

    gl_FragColor = vec4( mixture, 0.0 );
}

