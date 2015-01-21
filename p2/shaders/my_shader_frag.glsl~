varying vec3 normal;
varying vec3 cam_dir;
varying vec3 color;
uniform samplerCube cubemap;

void main(void)
{
    // A factor of color value related to the model view of the object 
    vec3 modelViewFactor = gl_ModelViewMatrix * vec4( 1.0, 1.0, 1.0, 1.0 );

    // Light Spot factor
    vec3 spotFactor = normal * cam_dir;

    // Reflection factor
    vec3 reflectedDirection = reflect(cam_dir, normalize(normal));

    // Mix all the factors together
    vec3 mixture = 0.05 * modelViewFactor + 0.05 * spotFactor + 0.9 * textureCube(cubemap, reflectedDirection);

    gl_FragColor = vec4( mixture, 0.0 );
}

