import pygame
import moderngl
import numpy as np

pygame.init()
info = pygame.display.Info()
size = (info.current_w, info.current_h)
pygame.display.set_mode(size, pygame.OPENGL | pygame.DOUBLEBUF | pygame.NOFRAME)
ctx = moderngl.create_context()
ctx.viewport = (0, 0, size[0], size[1])

vertex_shader = '''
#version 330
in vec2 in_vert;
out vec2 uv;
void main() {
    uv = in_vert * 0.5 + 0.5;
    gl_Position = vec4(in_vert, 0.0, 1.0);
}
'''

fragment_shader = '''
#version 330
in vec2 uv;
out vec4 fragColor;
uniform float time;

vec2 barrel(vec2 p) {
    p = p * 2.0 - 1.0;
    p *= 0.91;
    float r2 = p.x * p.x + p.y * p.y;
    p *= 1.0 + 0.15 * r2;
    return p * 0.5 + 0.5;
}

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

void main() {
    vec2 bp = barrel(uv);

    if (bp.x < 0.0 || bp.x > 1.0 || bp.y < 0.0 || bp.y > 1.0) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // calm dark background, just a hint of green, no grid, no bright lines
    float baseTint = 0.095;
    float noise = hash(bp * 500.0 + time * 0.3) * 0.006;

    // static scanlines (texture only, not moving)
    float scan = sin(bp.y * 800.0) * 0.015;

    float vign = 1.0 - dot(bp - 0.5, bp - 0.5) * 0.9;

    float green = (baseTint + scan + noise) * vign;
    fragColor = vec4(green * 0.15, green * 1.3, green * 0.5, 1.0);
}
'''

prog = ctx.program(vertex_shader=vertex_shader, fragment_shader=fragment_shader)

vertices = np.array([-1, -1, 1, -1, -1, 1, 1, 1], dtype='f4')
vbo = ctx.buffer(vertices.tobytes())
vao = ctx.simple_vertex_array(prog, vbo, 'in_vert', mode=moderngl.TRIANGLE_STRIP)

clock = pygame.time.Clock()
t = 0.0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

    ctx.clear(0.0, 0.0, 0.0)
    prog['time'].value = t
    vao.render()
    pygame.display.flip()
    t += clock.tick(60) / 1000.0

pygame.quit()